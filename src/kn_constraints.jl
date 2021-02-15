# Constraints utilities


##################################################
# Constraint definition
"Add constraint to model."
function KN_add_cons(m::Model, ncons::Integer)
    ptr_cons = zeros(Cint, ncons)
    ret = @kn_ccall(add_cons, Cint, (Ptr{Cvoid}, Cint, Ptr{Cint}), m.env, ncons, ptr_cons)
    _checkraise(ret)
    return ptr_cons
end

function KN_add_con(m::Model)
    ptr_cons = Cint[0]
    ret = @kn_ccall(add_con, Cint, (Ptr{Cvoid}, Ptr{Cint}), m.env, ptr_cons)
    _checkraise(ret)
    return ptr_cons[1]
end


##################################################
# Setters
##################################################
@define_setters set_con_eqbnd Cdouble
@define_setters set_con_lobnd Cdouble
@define_setters set_con_upbnd Cdouble
@define_setters set_con_dual_init_value Cdouble
@define_setters set_con_scaling Cdouble
@define_setters set_con_feastol Cdouble
@define_setters set_con_property Cint
@define_setters add_con_constant Cdouble


##################################################
# Getters
##################################################
if KNITRO_VERSION >= v"12.0"
    @define_getters get_con_lobnds Cdouble
    @define_getters get_con_upbnds Cdouble
    @define_getters get_con_eqbnds Cdouble
end

##################################################

##################################################
# Constraint structure
##################################################
#------------------------------
# add structure of linear constraint
#------------------------------
"""
Add linear structure to the constraint unctions.
Each component i of arrays indexCons, indexVars and coefs adds a linear
term:
   coefs[i]*x[indexVars[i]]
to constraint c[indexCons[i]].

"""
function KN_add_con_linear_struct(m::Model,
                                  jacIndexCons::Vector{Cint},
                                  jacIndexVars::Vector{Cint},
                                  jacCoefs::Vector{Cdouble})
    # get number of constraints
    nnz = length(jacIndexCons)
    @assert nnz == length(jacIndexVars) == length(jacCoefs)
    ret = @kn_ccall(add_con_linear_struct,
                    Cint,
                    (Ptr{Cvoid}, KNLONG, Ptr{Cint}, Ptr{Cint}, Ptr{Cdouble}),
                    m.env,
                    nnz,
                    jacIndexCons,
                    jacIndexVars,
                    jacCoefs)
    _checkraise(ret)
end

function KN_add_con_linear_struct(m::Model,
                                  indexCon::Integer,
                                  indexVar::Vector{Cint},
                                  coefs::Vector{Cdouble})
    # get number of constraints
    nnz = length(indexVar)
    @assert nnz == length(coefs)
    ret = @kn_ccall(add_con_linear_struct_one,
                    Cint,
                    (Ptr{Cvoid}, KNLONG, Cint, Ptr{Cint}, Ptr{Cdouble}),
                    m.env,
                    nnz,
                    indexCon,
                    indexVar,
                    coefs)
    _checkraise(ret)
end
KN_add_con_linear_struct(m::Model, indexCon::Integer, indexVar::Integer, coef::Cdouble) =
    KN_add_con_linear_struct(m, indexCon, Cint[indexVar], [coef])

#------------------------------
# add constraint quadratic structure
#------------------------------
"""
Add quadratic structure to the constraint functions.
Each component i of arrays indexCons, indexVars1, indexVars2 and coefs adds a
quadratic term:
   coefs[i]*x[indexVars1[i]]*x[indexVars2[i]]
to the constraint c[indexCons[i]].

"""
function KN_add_con_quadratic_struct(m::Model,
                                     indexCons::Vector{Cint},
                                     indexVars1::Vector{Cint},
                                     indexVars2::Vector{Cint},
                                     coefs::Vector{Cdouble})
    # get number of constraints
    nnz = length(indexVars1)
    @assert nnz == length(indexCons) == length(indexVars2) == length(coefs)
    ret = @kn_ccall(add_con_quadratic_struct,
                    Cint,
                    (Ptr{Cvoid}, KNLONG, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}, Ptr{Cdouble}),
                    m.env,
                    nnz,
                    indexCons,
                    indexVars1,
                    indexVars2,
                    coefs)
    _checkraise(ret)
end

function KN_add_con_quadratic_struct(m::Model,
                                     indexCons::Integer,
                                     indexVars1::Vector{Cint},
                                     indexVars2::Vector{Cint},
                                     coefs::Vector{Cdouble})
    # get number of constraints
    nnz = length(indexVars1)
    @assert nnz == length(indexVars2) == length(coefs)
    ret = @kn_ccall(add_con_quadratic_struct_one,
                    Cint,
                    (Ptr{Cvoid}, KNLONG, Cint, Ptr{Cint}, Ptr{Cint}, Ptr{Cdouble}),
                    m.env,
                    nnz,
                    indexCons,
                    indexVars1,
                    indexVars2,
                    coefs)
    _checkraise(ret)
end
KN_add_con_quadratic_struct(m, indexCons::Integer, indexVar1::Integer, indexVar2::Integer, coef::Cdouble) =
KN_add_con_quadratic_struct(m, indexCons, Cint[indexVar1], Cint[indexVar2], Cdouble[coef])

#------------------------------
# Conic structure
#------------------------------
"""
Add L2 norm structure of the form ||Ax + b||_2 to a constraint.
  indexCon:    The constraint index that the L2 norm term will be added to.
  nCoords:     The number of rows in "A" (or dimension of "b")
  nnz:         The number of sparse non-zero elements in "A"
  indexCoords: The coordinate (row) index for each non-zero element in "A".
  indexVars:   The variable (column) index for each non-zero element in "A"
  coefs:       The coefficient value for each non-zero element in "A"
  constants:   The array "b" - may be set to NULL to ignore "b"

#Note
L2 norm structure can currently only be added to constraints that
otherwise only have linear (or constant) structure.  In this way
they can be used to define conic constraints of the form
||Ax + b|| <= c'x + d.  The "c" coefficients should be added through
"KN_add_con_linear_struct()" and "d" can be set as a constraint bound
or through "KN_add_con_constants()".

#Note
Models with L2 norm structure are currently only handled by the
Interior/Direct (KN_ALG_BAR_DIRECT) algorithm in Knitro.  Any model
with structure defined with KN_add_L2norm() will automatically be
forced to use this algorithm.

"""
function KN_add_con_L2norm(m::Model, indexCon::Integer, nCoords::Integer, nnz::Integer,
                       indexCoords::Vector{Cint}, indexVars::Vector{Cint},
                       coefs::Vector{Cdouble}, constants::Vector{Cdouble})
    @assert length(coefs) == length(indexVars) == length(indexCoords) == nnz
    ret = @kn_ccall(add_con_L2norm,
                    Cint,
                    (Ptr{Cvoid}, Cint, Cint, KNLONG, Ptr{Cint}, Ptr{Cint},
                     Ptr{Cdouble}, Ptr{Cdouble}),
                    m.env, indexCon, nCoords, nnz, indexCoords,
                    indexVars, coefs, constants)
    _checkraise(ret)
end

##################################################
# Complementary constraints
##################################################
"""
This function adds complementarity constraints to the problem.
The two lists are of equal length, and contain matching pairs of
variable indices.  Each pair defines a complementarity constraint
between the two variables.  The function can only be called once.
The array "ccTypes" specifies the type of complementarity:
   KN_CCTYPE_VARVAR: two (non-negative) variables
   KN_CCTYPE_VARCON: a variable and a constraint
   KN_CCTYPE_CONCON: two constraints

#Note
Currently only KN_CCTYPE_VARVAR is supported.  The other
"ccTypes" will be added in future releases.

"""
function KN_set_compcons(m::Model,
                         ccTypes::Vector{Cint},
                         indexComps1::Vector{Cint},
                         indexComps2::Vector{Cint})
    # get number of constraints
    nnc = length(ccTypes)
    @assert nnc == length(indexComps1) == length(indexComps2)
    ret = @kn_ccall(set_compcons,
                    Cint,
                    (Ptr{Cvoid}, Cint, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}),
                    m.env,
                    nnc,
                    ccTypes,
                    indexComps1,
                    indexComps2)
    _checkraise(ret)
end

@define_setters set_compcon_scaling Cdouble
@define_setters set_compcon_feastol Cdouble


##################################################
## Naming constraints
##################################################
function KN_set_con_names(m::Model, nindex::Integer, name::String)
    ret = @kn_ccall(set_con_name, Cint,
                    (Ptr{Cvoid}, Cint, Ptr{Cchar}),
                    m.env, nindex, name)
    _checkraise(ret)
end

function KN_set_con_names(m::Model, conIndex::Vector{Cint}, names::Vector{String})
    ncon = length(conIndex)
    ret = @kn_ccall(set_con_names, Cint,
                    (Ptr{Cvoid}, Cint, Ptr{Cint}, Ptr{Ptr{Char}}),
                    m.env, ncon, conIndex, names)
    _checkraise(ret)
end

function KN_set_con_names(m::Model, names::Vector{String})
    ret = @kn_ccall(set_con_names_all, Cint, (Ptr{Cvoid}, Ptr{Ptr{Cchar}}),
                    m.env, names)
    _checkraise(ret)
end

function KN_set_compcon_names(m::Model, nindex::Integer, name::String)
    ret = @kn_ccall(set_compcon_name, Cint,
                    (Ptr{Cvoid}, Cint, Ptr{Cchar}),
                    m.env, nindex, name)
    _checkraise(ret)
end

function KN_set_compcon_names(m::Model, conIndex::Vector{Cint}, names::Vector{String})
    ncon = length(conIndex)
    ret = @kn_ccall(set_compcon_names, Cint,
                    (Ptr{Cvoid}, Cint, Ptr{Cint}, Ptr{Ptr{Char}}),
                    m.env, ncon, conIndex, names)
    _checkraise(ret)
end

function KN_set_compcon_names(m::Model, names::Vector{String})
    ret = @kn_ccall(set_compcon_names_all, Cint, (Ptr{Cvoid}, Ptr{Ptr{Cchar}}),
                    m.env, names)
    _checkraise(ret)
end

# Getters
if KNITRO_VERSION >= v"12.0"
    function KN_get_con_names(m::Model, max_length=1024)
        return String[KN_get_con_names(m, Cint(id-1), max_length) for id in 1:KN_get_number_cons(m)]
    end

    function KN_get_con_names(m::Model, index::Vector{Cint}, max_length=1024)
        return String[KN_get_con_names(m, id, max_length) for id in index]
    end

    function KN_get_con_names(m::Model, index::Cint, max_length=1024)
        rawname = zeros(Cchar, max_length)
        ret = @kn_ccall(get_con_name, Cint,
                        (Ptr{Cvoid}, Cint, Cint, Ptr{Cchar}),
                        m.env, index, max_length, rawname)
        _checkraise(ret)
        name = String(strip(String(convert(Vector{UInt8}, rawname)), '\0'))
        return name
    end
end

