# Common functions


# KNITRO special types
const KNLONG = Clonglong
const KNBOOL = Cint


"A macro to make calling KNITRO's KN_* C API a little cleaner"
macro kn_ccall(func, args...)
    f = Base.Meta.quot(Symbol("KN_$(func)"))
    args = [esc(a) for a in args]
    quote
        ccall(($f, libknitro), $(args...))
    end
end

macro define_getters(function_name, type)
    fname = Symbol("KN_" * string(function_name))
    quote
        function $(esc(fname))(kc::Model, index::Vector{Cint})
            result = zeros($type, length(index))
            ret = @kn_ccall($function_name, Cint,
                            (Ptr{Cvoid}, Cint, Ptr{Cint}, Ptr{$type}),
                            kc.env, length(index), index, result)
            _checkraise(ret)
            return result
        end
    end
end

macro define_setters(function_name, type)
    name_singular = string(function_name)
    # Names of functions in Julia wrapper
    if endswith(name_singular, 'y')  # plural of "y" is "ies"
        name_plural = name_singular[1:end-1] * "ies"
    else
        name_plural = name_singular * "s"
    end
    fname      = Symbol("KN_" * name_plural)
    fnameshort = Symbol("KN_" * name_singular)
    # Names of C function in knitro.h
    c_fname      = Symbol(name_singular)
    c_fnames     = Symbol(name_plural)
    c_fnames_all = Symbol(name_plural * "_all")

    quote
        function $(esc(fname))(kc::Model, values::Vector{$type})
            ret = @kn_ccall($(c_fnames_all), Cint,
                            (Ptr{Cvoid}, Ptr{$type}),
                            kc.env, values)
            _checkraise(ret)
            return
        end
        function $(esc(fname))(kc::Model, index::Vector{Cint}, values::Vector{$type})
            @assert length(index) == length(values)
            ret = @kn_ccall($(c_fnames), Cint,
                            (Ptr{Cvoid}, Cint, Ptr{Cint}, Ptr{$type}),
                            kc.env, length(index), index, values)
            _checkraise(ret)
            return
        end
        function $(esc(fname))(kc::Model, index::Integer, value::$type)
            ret = @kn_ccall($(c_fname), Cint,
                            (Ptr{Cvoid}, Cint, $type),
                            kc.env, index, value)
            _checkraise(ret)
            return
        end
        $(esc(fnameshort))(kc::Model, index::Integer, value::$type) = $(esc(fname))(kc, index, value)
    end
end

macro kn_set(function_name, type)
    fname = Symbol("KN_" * string(function_name))
    quote
        function $(esc(fname))(m::Model, val::$type)
            ret = @kn_ccall($function_name, Cint, (Ptr{Cvoid}, $type),
                            m.env, val)
            _checkraise(ret)
        end
    end
end

macro kn_get(function_name, type)
    fname = Symbol("KN_" * string(function_name))
    quote
        function $(esc(fname))(m::Model)
            val = zeros($type, 1)
            ret = @kn_ccall($function_name, Cint, (Ptr{Cvoid}, Ptr{$type}),
                            m.env, val)
            _checkraise(ret)
            return val[1]
        end
    end
end

"Check if return value is valid."
function _checkraise(ret::Cint)
    if ret != 0
        error("Fail to use specified function: $ret")
    end
end

"Format output returned by KNITRO as proper Julia string."
function format_output(output::AbstractString)
    # remove trailing whitespace
    res = strip(output)
    # remove special characters
    res = strip(res, '\0')
end

"Return the current KNITRO version."
function get_release()
    len = 15
    out = zeros(Cchar,len)

    @kn_ccall(get_release, Cvoid, (Cint, Ptr{Cchar}), len, out)
    return String(strip(String(convert(Vector{UInt8},out)), '\0'))
end
