# Variables utilities

"Add variable to model."
function KN_add_var(m::Model)
    ptr_int = Cint[0]
    ret = @kn_ccall(add_var, Cint, (Ptr{Cvoid}, Ptr{Cint}), m.env, ptr_int)
    _checkraise(ret)
    return ptr_int[1]
end


function KN_add_vars(m::Model, nvars::Int)
    ptr_int = zeros(Cint, nvars)
    ret = @kn_ccall(add_vars, Cint, (Ptr{Cvoid}, Cint, Ptr{Cint}), m.env, nvars, ptr_int)
    _checkraise(ret)
    return ptr_int
end

##################################################
## Setters
##################################################
@define_setters set_var_lobnd Cdouble
@define_setters set_var_upbnd Cdouble
@define_setters set_var_fxbnd Cdouble
@define_setters set_var_type Cint
@define_setters set_var_property Cint
@define_setters set_var_honorbnd Cint
@define_setters set_var_feastol Cdouble

@define_setters set_var_primal_init_value Cdouble
@define_setters set_var_dual_init_value Cdouble


##################################################
# Getters
##################################################
if KNITRO_VERSION >= v"12.0"
    @define_getters get_var_lobnds Cdouble
    @define_getters get_var_upbnds Cdouble
    @define_getters get_var_eqbnds Cdouble
    @define_getters get_var_fxbnds Cdouble
end
if KNITRO_VERSION >= v"12.3"
    @define_getters get_var_types Cint
end

##################################################
## Naming variables
##################################################
"""
Set names for model components passed in by the user/modeling
language so that Knitro can internally print out these names.
Knitro makes a local copy of all inputs, so the application may
free memory after the call.
"""
function KN_set_var_names(m::Model, nindex::Integer, name::String)
    ret = @kn_ccall(set_var_name, Cint,
                    (Ptr{Cvoid}, Cint, Ptr{Cchar}),
                    m.env, nindex, name)
    _checkraise(ret)
end

function KN_set_var_names(m::Model, varIndex::Vector{Cint}, names::Vector{String})
    nvar = length(varIndex)
    ret = @kn_ccall(set_var_names, Cint,
                    (Ptr{Cvoid}, Cint, Ptr{Cint}, Ptr{Ptr{Char}}),
                    m.env, nvar, varIndex, names)
    _checkraise(ret)
end

function KN_set_var_names(m::Model, names::Vector{String})
    ret = @kn_ccall(set_var_names_all, Cint, (Ptr{Cvoid}, Ptr{Ptr{Cchar}}),
                    m.env, names)
    _checkraise(ret)
end

# Getters
if KNITRO_VERSION >= v"12.0"
    function KN_get_var_names(m::Model, max_length=1024)
        return String[KN_get_var_names(m, Cint(id-1), max_length) for id in 1:KN_get_number_vars(m)]
    end

    function KN_get_var_names(m::Model, index::Vector{Cint}, max_length=1024)
        return String[KN_get_var_names(m, id, max_length) for id in index]
    end

    function KN_get_var_names(m::Model, index::Cint, max_length=1024)
        rawname = zeros(Cchar, max_length)
        ret = @kn_ccall(get_var_name, Cint,
                        (Ptr{Cvoid}, Cint, Cint, Ptr{Cchar}),
                        m.env, index, max_length, rawname)
        _checkraise(ret)
        name = String(strip(String(convert(Vector{UInt8}, rawname)), '\0'))
        return name
    end
end


##################################################
## Scalings
##################################################
"""
Set an array of variable scaling and centering values to
perform a linear scaling
  x[i] = xScaleFactors[i] * xScaled[i] + xScaleCenters[i]
for each variable. These scaling factors should try to
represent the "typical" values of the "x" variables so that the
scaled variables ("xScaled") used internally by Knitro are close
to one.  The values for xScaleFactors should be positive.
If a non-positive value is specified, that variable will not
be scaled.
"""
function KN_set_var_scalings(m::Model, nindex::Integer,
                             xScaleFactors::Cdouble, xScaleCenters::Cdouble)
    ret = @kn_ccall(set_var_scaling, Cint,
                    (Ptr{Cvoid}, Cint, Cdouble, Cdouble),
                    m.env, nindex, xScaleFactors, xScaleCenters)
    _checkraise(ret)
end
KN_set_var_scalings(m::Model, nindex::Integer, xScaleFactors::Cdouble) =
    KN_set_var_scalings(m, nindex, xScaleFactors, 0.)


function KN_set_var_scalings(m::Model, valindex::Vector{Cint},
                             xScaleFactors::Vector{Cdouble}, xScaleCenters::Vector{Cdouble})
    nvar = length(valindex)
    @assert nvar = length(xScaleFactors) == length(xScaleCenters)
    ret = @kn_ccall(set_var_scalings, Cint,
                    (Ptr{Cvoid}, Cint, Ptr{Cint}, Ptr{Cdouble}, Ptr{Cdouble}),
                    m.env, nvar, valindex, xScaleFactors, xScaleCenters)
    _checkraise(ret)
end
KN_set_var_scalings(m::Model, xIndex::Vector{Cint}, xScaleFactors::Vector{Cdouble}) =
    KN_set_var_scalings(m, xIndex, xScaleFactors, zeros(Cdouble, length(xScaleFactors)))

function KN_set_var_scalings(m::Model,
                             xScaleFactors::Vector{Cdouble}, xScaleCenters::Vector{Cdouble})
    @assert length(xScaleFactors) == length(xScaleCenters)
    ret = @kn_ccall(set_var_scalings_all, Cint, (Ptr{Cvoid}, Ptr{Cdouble}, Ptr{Cdouble}),
                    m.env, xScaleFactors, xScaleCenters)
    _checkraise(ret)
end
KN_set_var_scalings(m::Model, xScaleFactors::Vector{Cdouble}) =
    KN_set_var_scalings(m, xScaleFactors, zeros(Cdouble, length(xScaleFactors)))

