# Knitro model attributes


##################################################
# objective
##################################################
# set objective sense
@kn_set set_obj_goal Cint

"""
Add linear structure to the objective function.
Each component i of arrays indexVars and coefs adds a linear term
   coefs[i]*x[indexVars[i]]
to the objective.

"""
function KN_add_obj_linear_struct(m::Model,
                                  objIndices::Vector{Cint},
                                  objCoefs::Vector{Cdouble})
    nnz = length(objIndices)
    @assert nnz == length(objCoefs)
    ret = @kn_ccall(add_obj_linear_struct, Cint,
                    (Ptr{Cvoid}, KNLONG, Ptr{Cint}, Ptr{Cdouble}),
                    m.env,
                    nnz,
                    objIndices,
                    objCoefs)
    _checkraise(ret)
end
KN_add_obj_linear_struct(m::Model, objindex::Int, objCoefs::Cdouble) =
    KN_add_obj_linear_struct(m, Cint[objindex], [objCoefs])

# quadratic part of objective
"""
Add quadratic structure to the objective function.
Each component i of arrays indexVars1, indexVars2 and coefs adds a quadratic
term
   coefs[i]*x[indexVars1[i]]*x[indexVars2[i]]
to the objective.

Note: if indexVars2[i] is < 0 then it adds a linear term
      coefs[i]*x[indexVars1[i]] instead.

"""
function KN_add_obj_quadratic_struct(m::Model,
                                     indexVars1::Vector{Cint},
                                     indexVars2::Vector{Cint},
                                     coefs::Vector{Cdouble})
    nnz = length(indexVars1)
    @assert nnz == length(indexVars2) == length(coefs)
    ret = @kn_ccall(add_obj_quadratic_struct, Cint,
                    (Ptr{Cvoid}, KNLONG, Ptr{Cint}, Ptr{Cint}, Ptr{Cdouble}),
                    m.env,
                    nnz,
                    indexVars1,
                    indexVars2,
                    coefs)
    _checkraise(ret)
end

@kn_set add_obj_constant Cdouble
@kn_set set_obj_scaling Cdouble


# Specify some properties of the objective and constraint functions.
# Note: use bit-wise specification of the features:
# bit value   meaning
#   0     1   KN_OBJ_CONVEX/KN_CON_CONVEX
#   1     2   KN_OBJ_CONCAVE/KN_CON_CONCAVE
#   2     4   KN_OBJ_CONTINUOUS/KN_CON_CONTINUOUS
#   3     8   KN_OBJ_DIFFERENTIABLE/KN_CON_DIFFERENTIABLE
#   4    16   KN_OBJ_TWICE_DIFFERENTIABLE/KN_CON_TWICE_DIFFERENTIABLE
#   5    32   KN_OBJ_NOISY/KN_CON_NOISY
#   6    64   KN_OBJ_NONDETERMINISTIC/KN_CON_NONDETERMINISTIC
@kn_set set_obj_property Cint

function KN_set_obj_name(m::Model, name::AbstractString)
    ret = @kn_ccall(set_obj_name, Cint, (Ptr{Cvoid}, Ptr{Cchar}),
                    m.env, name)
    _checkraise(ret)
end

##################################################
# Generic getters
##################################################
@kn_get get_number_vars Cint
@kn_get get_number_cons Cint
@kn_get get_number_rsds Cint
if KNITRO_VERSION >= v"12.3"
    @kn_get get_number_compcons Cint
end
@kn_get get_obj_value Cdouble
@kn_get get_obj_type Cint


##################################################
# Constraints getters
##################################################
function KN_get_con_values(m::Model)
    nc = KN_get_number_cons(m)
    consvals = zeros(Cdouble, nc)
    ret = @kn_ccall(get_con_values_all, Cint, (Ptr{Cvoid}, Ptr{Cdouble}),
                    m.env, consvals)
    _checkraise(ret)
    return consvals
end

function KN_get_con_values(m::Model, cIndex::Integer)
    nc = 1
    consvals = zeros(Cdouble, nc)
    ret = @kn_ccall(get_con_value, Cint, (Ptr{Cvoid}, Cint, Ptr{Cdouble}),
                    m.env, cIndex, consvals)
    _checkraise(ret)
    return consvals[1]
end

function KN_get_con_types(m::Model)
    nc = KN_get_number_cons(m)
    constypes = zeros(Cint, nc)
    ret = @kn_ccall(get_con_types_all, Cint, (Ptr{Cvoid}, Ptr{Cint}),
                    m.env, constypes)
    _checkraise(ret)
    return constypes
end

##################################################
# Continuous optimization results
##################################################
# Return the number of iterations made by KN_solve in "numIters".
@kn_get get_number_iters Cint

# Return the number of conjugate gradient (CG) iterations made by
# KN_solve in "numCGiters".
@kn_get get_number_cg_iters Cint

# Return the absolute feasibility error at the solution in "absFeasError".
# Refer to the Knitro manual section on Termination Tests for a
# detailed definition of this quantity.
@kn_get get_abs_feas_error Cdouble

# Return the relative feasibility error at the solution in "relFeasError".
# Refer to the Knitro manual section on Termination Tests for a
# detailed definition of this quantity.
@kn_get get_rel_feas_error Cdouble

# Return the absolute optimality error at the solution in "absOptError".
# Refer to the Knitro manual section on Termination Tests for a
# detailed definition of this quantity.
@kn_get get_abs_opt_error Cdouble

# Return the relative optimality error at the solution in "relOptError".
# Refer to the Knitro manual section on Termination Tests for a
# detailed definition of this quantity.
@kn_get get_rel_opt_error Cdouble

# Objective gradient
@kn_get get_objgrad_nnz Cint

function KN_get_objgrad_values(m::Model)
    nnz = KN_get_objgrad_nnz(m)
    indexVars = zeros(Cint, nnz)
    objGrad = zeros(Cdouble, nnz)
    ret = @kn_ccall(get_objgrad_values, Cint,
                    (Ptr{Cvoid}, Ptr{Cint}, Ptr{Cdouble}),
                    m.env, indexVars, objGrad)
    _checkraise(ret)
    return indexVars, objGrad
end

#--------------------
# Jacobian
#--------------------
@kn_get get_jacobian_nnz KNLONG

function KN_get_jacobian_values(m::Model)
    nnz = KN_get_jacobian_nnz(m)
    jacvars = zeros(Cint, nnz)
    jaccons = zeros(Cint, nnz)
    jaccoef = zeros(Cdouble, nnz)
    ret = @kn_ccall(get_jacobian_values, Cint,
                    (Ptr{Cvoid}, Ptr{Cint}, Ptr{Cint}, Ptr{Cdouble}),
                    m.env, jacvars, jaccons, jaccoef)
    _checkraise(ret)
    return jacvars, jaccons, jaccoef
end

# Rsd Jacobian
@kn_get get_rsd_jacobian_nnz KNLONG

"""
Return the values of the residual Jacobian in "indexRsds", "indexVars",
and "rsdJac".  The Jacobian values returned correspond to the non-zero
sparse Jacobian indices provided by the user.

"""
function KN_get_rsd_jacobian_values(m::Model)
    nnz = KN_get_rsd_jacobian_nnz(m)
    jacvars = zeros(Cint, nnz)
    jaccons = zeros(Cint, nnz)
    jaccoef = zeros(Cdouble, nnz)
    ret = @kn_ccall(get_rsd_jacobian_values, Cint,
                    (Ptr{Cvoid}, Ptr{Cint}, Ptr{Cint}, Ptr{Cdouble}),
                    m.env, jacvars, jaccons, jaccoef)
    _checkraise(ret)
    return jacvars, jaccons, jaccoef
end

# Hessian
@kn_get get_hessian_nnz KNLONG

function KN_get_hessian_values(m::Model)
    nnz = KN_get_hessian_nnz(m)
    indexVars1 = zeros(Cint, nnz)
    indexVars2 = zeros(Cint, nnz)
    hess = zeros(Cdouble, nnz)
    ret = @kn_ccall(get_hessian_values, Cint,
                    (Ptr{Cvoid}, Ptr{Cint}, Ptr{Cint}, Ptr{Cdouble}),
                    m.env, indexVars1, indexVars2, hess)
    _checkraise(ret)
    return indexVars1, indexVars2, hess
end

# Getters for CPU time are implemented only for Knitro version >= 12.0.
if KNITRO_VERSION >= v"12.0"
    @kn_get get_solve_time_cpu Cdouble
    @kn_get get_solve_time_real Cdouble
end

##################################################
# MIP utils
##################################################
# Return the number of nodes processed in the MIP solve
# in "numNodes".
@kn_get get_mip_number_nodes Cint

# Return the number of continuous subproblems processed in the
# MIP solve in "numSolves".
@kn_get get_mip_number_solves Cint

# Return the final absolute integrality gap in the MIP solve
# in "absGap". Refer to the Knitro manual section on Termination
# Tests for a detailed definition of this quantity. Set to
# KN_INFINITY if no incumbent (i.e., integer feasible) point found.
@kn_get get_mip_abs_gap Cdouble

# Return the final absolute integrality gap in the MIP solve
# int "relGap". Refer to the Knitro manual section on Termination
# Tests for a detailed definition of this quantity.  Set to
# KN_INFINITY if no incumbent (i.e., integer feasible) point found.
@kn_get get_mip_rel_gap Cdouble

# Return the objective value of the MIP incumbent solution in
# "incumbentObj". Set to KN_INFINITY if no incumbent (i.e., integer
# feasible) point found.
@kn_get get_mip_incumbent_obj Cdouble

# Return the value of the current MIP relaxation bound in "relaxBound".
@kn_get get_mip_relaxation_bnd Cdouble

# Return the objective value of the most recently solved MIP
# node subproblem in "lastNodeObj".
@kn_get get_mip_lastnode_obj Cdouble

# Return the MIP incumbent solution in 'x' if one exists.
@kn_get get_mip_incumbent_x Cdouble

# Set the branching priorities for integer variables. Must first
# set the types of variables (e.g. by calling KN_set_var_types) before
# calling this function. Priorities must be positive numbers
# (variables with non-positive values are ignored).  Variables with
# higher priority values will be considered for branching before
# variables with lower priority values.  When priorities for a subset
# of variables are equal, the branching rule is applied as a tiebreaker.
# Values for continuous variables are ignored.  Knitro makes a local
# copy of all inputs, so the application may free memory after the call.
@define_setters set_mip_branching_prioritie Cint

# Set strategies for dealing with individual integer variables. Possible
# strategy values include:
#   KN_MIP_INTVAR_STRATEGY_NONE    0 (default)
#   KN_MIP_INTVAR_STRATEGY_RELAX   1
#   KN_MIP_INTVAR_STRATEGY_MPEC    2 (binary variables only)
# indexVars should be an index value corresponding to an integer variable
# (nothing is done if the index value corresponds to a continuous variable),
# and xStrategies should correspond to one of the strategy values listed above.
@define_setters set_mip_intvar_strategie Cint


##################################################
# Parameters
##################################################
#------------------------------
# Setters
#------------------------------
# Int params
function KN_set_param(m::Model, id::Integer, value::Integer)
    ret = @kn_ccall(set_int_param, Cint, (Ptr{Cvoid}, Cint, Cint),
                    m.env, id, value)
    _checkraise(ret)
end

function KN_set_param(m::Model, param::AbstractString, value::Integer)
    ret = @kn_ccall(set_int_param_by_name, Cint, (Ptr{Cvoid}, Ptr{Cchar}, Cint),
                    m.env, param, value)
    _checkraise(ret)
end

# Double params
function KN_set_param(m::Model, id::Integer, value::Cdouble)
    ret = @kn_ccall(set_double_param, Cint, (Ptr{Cvoid}, Cint, Cdouble),
                    m.env, id, value)
    _checkraise(ret)
end

function KN_set_param(m::Model, param::AbstractString, value::Cdouble)
    ret = @kn_ccall(set_double_param_by_name, Cint, (Ptr{Cvoid}, Ptr{Cchar}, Cdouble),
                    m.env, param, value)
    _checkraise(ret)
end

# Char params
function KN_set_param(m::Model, id::Integer, value::AbstractString)
    ret = @kn_ccall(set_char_param, Cint, (Ptr{Cvoid}, Cint, Ptr{Cchar}),
                    m.env, id, value)
    _checkraise(ret)
end

function KN_set_param(m::Model, param::AbstractString, value::AbstractString)
    ret = @kn_ccall(set_char_param_by_name, Cint, (Ptr{Cvoid}, Ptr{Cchar}, Ptr{Cchar}),
                    m.env, param, value)
    _checkraise(ret)
end

#------------------------------
# Getters
#------------------------------

# Int params
function KN_get_int_param(m::Model, id::Integer)
    res = Cint[0]
    ret = @kn_ccall(get_int_param, Cint, (Ptr{Cvoid}, Cint, Ptr{Cint}),
                    m.env, id, res)
    _checkraise(ret)
    return res[1]
end

function KN_get_int_param(m::Model, param::AbstractString)
    res = Cint[0]
    ret = @kn_ccall(get_int_param_by_name, Cint, (Ptr{Cvoid}, Ptr{Cchar}, Ptr{Cint}),
                    m.env, param, res)
    _checkraise(ret)
    return res[1]
end

# Double params
function KN_get_double_param(m::Model, id::Integer)
    res = Cdouble[0.]
    ret = @kn_ccall(get_double_param, Cint, (Ptr{Cvoid}, Cint, Ptr{Cdouble}),
                    m.env, id, res)
    _checkraise(ret)
    return res[1]
end

function KN_get_double_param(m::Model, param::AbstractString)
    res = Cdouble[0.]
    ret = @kn_ccall(get_double_param_by_name, Cint, (Ptr{Cvoid}, Ptr{Cchar}, Ptr{Cdouble}),
                    m.env, param, res)
    _checkraise(ret)
    return res[1]
end

#------------------------------
# Params information
#------------------------------
function KN_get_param_name(m::Model, id::Integer)
    output_size = 128
    res = " "^output_size
    ret = @kn_ccall(get_param_name, Cint, (Ptr{Cvoid}, Cint, Ptr{Cchar}, Csize_t),
                    m.env, id, res, output_size)
    _checkraise(ret)
    return format_output(res)
end

function KN_get_param_doc(m::Model, id::Integer)
    output_size = 128
    res = " "^output_size
    ret = @kn_ccall(get_param_doc, Cint, (Ptr{Cvoid}, Cint, Ptr{Cchar}, Csize_t),
                    m.env, id, res, output_size)
    _checkraise(ret)
    return format_output(res)
end

function KN_get_param_type(m::Model, id::Integer)
    res = Cint[0]
    ret = @kn_ccall(get_param_type, Cint, (Ptr{Cvoid}, Cint, Ptr{Cint}),
                    m.env, id, res)
    _checkraise(ret)
    return res[1]
end

function KN_get_num_param_values(m::Model, id::Integer)
    res = Cint[0]
    ret = @kn_ccall(get_num_param_values, Cint, (Ptr{Cvoid}, Cint, Ptr{Cint}),
                    m.env, id, res)
    _checkraise(ret)
    return res[1]
end

function KN_get_param_value_doc(m::Model, id::Integer, value_id::Integer)
    output_size = 128
    res = " "^output_size
    ret = @kn_ccall(get_param_value_doc, Cint,
                    (Ptr{Cvoid}, Cint, Cint, Ptr{Cchar}, Csize_t),
                    m.env, id, value_id, res, output_size)
    _checkraise(ret)
    return format_output(res)
end

function KN_get_param_id(m::Model, name::AbstractString)
    res = Cint[0]
    ret = @kn_ccall(get_param_id, Cint, (Ptr{Cvoid}, Ptr{Cchar}, Ptr{Cint}),
                    m.env, name, res)
    _checkraise(ret)
    return res[1]
end


function Base.show(io::IO, m::Model)
    if is_valid(m)
        println(io, "$(get_release())")
        println(io, "-----------------------")
        println(io, "Problem Characteristics")
        println(io, "-----------------------")
        println(io, "Objective goal:  Minimize")
        println(io, "Objective type:  $(KN_get_obj_type(m))")
        println(io, "Number of variables:                             $(KN_get_number_vars(m))")
        println(io, "Number of constraints:                           $(KN_get_number_cons(m))")
        println(io, "Number of nonzeros in Jacobian:                  $(KN_get_jacobian_nnz(m))")
        println(io, "Number of nonzeros in Hessian:                   $(KN_get_hessian_nnz(m))")

    else
        println(io, "KNITRO Problem: NULL")
    end
end

