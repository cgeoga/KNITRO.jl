# MathOptInterface results
MOI.get(model::Optimizer, ::MOI.RawStatusString) = string(get_status(model.inner))

KN_TO_MOI_RETURN_STATUS = Dict{Int,MOI.TerminationStatusCode}(
    0 => MOI.LOCALLY_SOLVED,
    -100 => MOI.ALMOST_OPTIMAL,
    -101 => MOI.SLOW_PROGRESS,
    -102 => MOI.SLOW_PROGRESS,
    -103 => MOI.SLOW_PROGRESS,
    -200 => MOI.LOCALLY_INFEASIBLE,
    -201 => MOI.LOCALLY_INFEASIBLE,
    -202 => MOI.LOCALLY_INFEASIBLE,
    -203 => MOI.LOCALLY_INFEASIBLE,
    -204 => MOI.LOCALLY_INFEASIBLE,
    -205 => MOI.LOCALLY_INFEASIBLE,
    -300 => MOI.DUAL_INFEASIBLE,
    -301 => MOI.DUAL_INFEASIBLE,
    -400 => MOI.ITERATION_LIMIT,
    -401 => MOI.TIME_LIMIT,
    -402 => MOI.OTHER_LIMIT,
    -403 => MOI.OTHER_LIMIT,
    -404 => MOI.OTHER_LIMIT,
    -405 => MOI.OTHER_LIMIT,
    -406 => MOI.NODE_LIMIT,
    -410 => MOI.ITERATION_LIMIT,
    -411 => MOI.TIME_LIMIT,
    -412 => MOI.INFEASIBLE,
    -413 => MOI.INFEASIBLE,
    -414 => MOI.OTHER_LIMIT,
    -415 => MOI.OTHER_LIMIT,
    -416 => MOI.NODE_LIMIT,
    -500 => MOI.INVALID_MODEL,
    -501 => MOI.NUMERICAL_ERROR,
    -502 => MOI.INVALID_MODEL,
    -503 => MOI.MEMORY_LIMIT,
    -504 => MOI.INTERRUPTED,
    -505 => MOI.OTHER_ERROR,
    -506 => MOI.OTHER_ERROR,
    -507 => MOI.OTHER_ERROR,
    -508 => MOI.OTHER_ERROR,
    -509 => MOI.OTHER_ERROR,
    -510 => MOI.OTHER_ERROR,
    -511 => MOI.OTHER_ERROR,
    -512 => MOI.OTHER_ERROR,
    -513 => MOI.OTHER_ERROR,
    -514 => MOI.OTHER_ERROR,
    -515 => MOI.OTHER_ERROR,
    -516 => MOI.OTHER_ERROR,
    -517 => MOI.OTHER_ERROR,
    -518 => MOI.OTHER_ERROR,
    -519 => MOI.OTHER_ERROR,
    -519 => MOI.OTHER_ERROR,
    -520 => MOI.OTHER_ERROR,
    -521 => MOI.OTHER_ERROR,
    -522 => MOI.OTHER_ERROR,
    -523 => MOI.OTHER_ERROR,
    -524 => MOI.OTHER_ERROR,
    -525 => MOI.OTHER_ERROR,
    -526 => MOI.OTHER_ERROR,
    -527 => MOI.OTHER_ERROR,
    -528 => MOI.OTHER_ERROR,
    -529 => MOI.OTHER_ERROR,
    -530 => MOI.OTHER_ERROR,
    -531 => MOI.OTHER_ERROR,
    -532 => MOI.OTHER_ERROR,
    -600 => MOI.OTHER_ERROR,
)

# Refer to KNITRO manual for solver status:
# https://www.artelys.com/tools/knitro_doc/3_referenceManual/returnCodes.html#returncodes
function MOI.get(model::Optimizer, ::MOI.TerminationStatus)
    if model.number_solved == 0
        return MOI.OPTIMIZE_NOT_CALLED
    end
    status = get_status(model.inner)
    if haskey(KN_TO_MOI_RETURN_STATUS, status)
        return KN_TO_MOI_RETURN_STATUS[status]
    else
        error("Unrecognized KNITRO status $status")
    end
end

# TODO
function MOI.get(model::Optimizer, ::MOI.ResultCount)
    return (model.number_solved >= 1) ? 1 : 0
end

function MOI.get(model::Optimizer, status::MOI.PrimalStatus)
    if (
        (model.number_solved == 0) ||   # no solution available if the model is unsolved
        (status.result_index > 1)       # KNITRO stores only a single solution
    )
        return MOI.NO_SOLUTION
    end
    status = get_status(model.inner)
    if status == 0
        return MOI.FEASIBLE_POINT
    elseif -109 <= status <= -100
        return MOI.FEASIBLE_POINT
    elseif -209 <= status <= -200
        return MOI.INFEASIBLE_POINT
    elseif status == -300
        return MOI.INFEASIBILITY_CERTIFICATE
    elseif -409 <= status <= -400
        return MOI.FEASIBLE_POINT
    elseif -419 <= status <= -410
        return MOI.INFEASIBLE_POINT
    elseif -599 <= status <= -500
        return MOI.UNKNOWN_RESULT_STATUS
    else
        return MOI.UNKNOWN_RESULT_STATUS
    end
end

function MOI.get(model::Optimizer, status::MOI.DualStatus)
    if (
        (model.number_solved == 0) ||   # no solution available if the model is unsolved
        (status.result_index > 1)       # KNITRO stores only a single solution
    )
        return MOI.NO_SOLUTION
    end
    status = get_status(model.inner)
    if status == 0
        return MOI.FEASIBLE_POINT
    elseif -109 <= status <= -100
        return MOI.FEASIBLE_POINT
    elseif -209 <= status <= -200
        return MOI.INFEASIBILITY_CERTIFICATE
    elseif status == -300
        return MOI.NO_SOLUTION
    elseif -409 <= status <= -400
        return MOI.FEASIBLE_POINT
    elseif -419 <= status <= -410
        return MOI.INFEASIBLE_POINT
    elseif -599 <= status <= -500
        return MOI.UNKNOWN_RESULT_STATUS
    else
        return MOI.UNKNOWN_RESULT_STATUS
    end
end

function MOI.get(model::Optimizer, obj::MOI.ObjectiveValue)
    if model.number_solved == 0
        error("ObjectiveValue not available.")
    elseif obj.result_index > 1
        throw(MOI.ResultIndexBoundsError{MOI.ObjectiveValue}(obj, 1))
    end
    return get_objective(model.inner)
end

function MOI.get(model::Optimizer, v::MOI.VariablePrimal, vi::MOI.VariableIndex)
    if model.number_solved == 0
        error("VariablePrimal not available.")
    elseif v.result_index > 1
        throw(MOI.ResultIndexBoundsError{MOI.VariablePrimal}(v, 1))
    end
    check_inbounds(model, vi)
    return get_solution(model.inner, vi.value)
end
function MOI.get(model::Optimizer, v::MOI.VariablePrimal, vi::Vector{MOI.VariableIndex})
    if model.number_solved == 0
        error("VariablePrimal not available.")
    elseif v.result_index > 1
        throw(MOI.ResultIndexBoundsError{MOI.VariablePrimal}(v, 1))
    end
    x = get_solution(model.inner)
    return [x[v.value] for v in vi]
end

macro checkcons(model, ci, cp)
    quote
        if $(esc(model)).number_solved == 0
            error("Solve problem before accessing solution.")
        elseif $(esc(cp)).result_index > 1
            throw(MOI.ResultIndexBoundsError{typeof($(esc(cp)))}($(esc(cp)), 1))
        end
        if !(0 <= $(esc(ci)).value <= number_constraints($(esc(model))) - 1)
            error("Invalid constraint index ", $(esc(ci)).value)
        end
    end
end

##################################################
## ConstraintPrimal
function MOI.get(
    model::Optimizer,
    cp::MOI.ConstraintPrimal,
    ci::MOI.ConstraintIndex{S,T},
) where {S<:SF,T<:SS}
    @checkcons(model, ci, cp)
    g = KN_get_con_values(model.inner)
    index = model.constraint_mapping[ci] .+ 1
    return g[index]
end

function MOI.get(
    model::Optimizer,
    cp::MOI.ConstraintPrimal,
    ci::MOI.ConstraintIndex{S,T},
) where {S<:VAF,T<:Union{MOI.Nonnegatives,MOI.Nonpositives}}
    @checkcons(model, ci, cp)
    g = KN_get_con_values(model.inner)
    index = model.constraint_mapping[ci] .+ 1
    return g[index]
end

function MOI.get(
    model::Optimizer,
    cp::MOI.ConstraintPrimal,
    ci::MOI.ConstraintIndex{S,T},
) where {S<:VOV,T<:Union{MOI.Nonnegatives,MOI.Nonpositives}}
    @checkcons(model, ci, cp)
    x = get_solution(model.inner)
    index = model.constraint_mapping[ci] .+ 1
    return x[index]
end

function MOI.get(
    model::Optimizer,
    cp::MOI.ConstraintPrimal,
    ci::MOI.ConstraintIndex{S,T},
) where {S<:Union{VAF,VOV},T<:MOI.Zeros}
    @checkcons(model, ci, cp)
    ncons = length(model.constraint_mapping[ci])
    return zeros(ncons)
end

function MOI.get(
    model::Optimizer,
    cp::MOI.ConstraintPrimal,
    ci::MOI.ConstraintIndex{S,T},
) where {S<:Union{VAF,VOV},T<:MOI.SecondOrderCone}
    @checkcons(model, ci, cp)
    x = get_solution(model.inner)
    index = model.constraint_mapping[ci] .+ 1
    return x[index]
end

function MOI.get(
    model::Optimizer,
    cp::MOI.ConstraintPrimal,
    ci::MOI.ConstraintIndex{MOI.VariableIndex,<:LS},
)
    if model.number_solved == 0
        error("ConstraintPrimal not available.")
    elseif cp.result_index > 1
        throw(MOI.ResultIndexBoundsError{MOI.ConstraintPrimal}(cp, 1))
    end
    vi = MOI.VariableIndex(ci.value)
    check_inbounds(model, vi)
    return get_solution(model.inner, vi.value)
end

function MOI.get(
    model::Optimizer,
    cp::MOI.ConstraintPrimal,
    ci::Vector{MOI.ConstraintIndex{MOI.VariableIndex,<:LS}},
)
    if model.number_solved == 0
        error("ConstraintPrimal not available.")
    elseif cp.result_index > 1
        throw(MOI.ResultIndexBoundsError{MOI.ConstraintPrimal}(cp, 1))
    end
    x = get_solution(model.inner)
    return [x[c.value] for c in ci]
end

##################################################
## ConstraintDual
#
# KNITRO's dual sign depends on optimization sense.
sense_dual(model::Optimizer) = (model.sense == MOI.MAX_SENSE) ? 1.0 : -1.0

function MOI.get(
    model::Optimizer,
    cd::MOI.ConstraintDual,
    ci::MOI.ConstraintIndex{S,T},
) where {S<:SF,T<:SS}
    @checkcons(model, ci, cd)

    index = model.constraint_mapping[ci] + 1
    lambda = get_dual(model.inner)
    return sense_dual(model) * lambda[index]
end

function MOI.get(
    model::Optimizer,
    cd::MOI.ConstraintDual,
    ci::MOI.ConstraintIndex{S,T},
) where {S<:VAF,T<:VLS}
    @checkcons(model, ci, cd)
    index = model.constraint_mapping[ci] .+ 1
    lambda = get_dual(model.inner)
    return sense_dual(model) * lambda[index]
end

function MOI.get(
    model::Optimizer,
    ::MOI.ConstraintDual,
    ci::MOI.ConstraintIndex{S,T},
) where {S<:VOV,T<:VLS}
    offset = number_constraints(model)
    index = model.constraint_mapping[ci] .+ 1 .+ offset
    lambda = get_dual(model.inner)
    return sense_dual(model) * lambda[index]
end

###
# Get constraint of a SOC constraint.
#
# Use the following mathematical property.  Let
#
#   ||u_i || <= t_i      with dual constraint    || z_i || <= w_i
#
# At optimality, we have
#
#   w_i * u_i  = - t_i z_i
#
###
function MOI.get(
    model::Optimizer,
    cd::MOI.ConstraintDual,
    ci::MOI.ConstraintIndex{S,T},
) where {S<:Union{VAF,VOV},T<:MOI.SecondOrderCone}
    @checkcons(model, ci, cd)
    index_var = model.constraint_mapping[ci] .+ 1
    index = model.constraint_mapping[ci] .+ 1
    index_con = ci.value + 1
    x = get_solution(model.inner)[index_var]
    # By construction.
    t_i = x[1]
    u_i = x[2:end]
    w_i = get_dual(model.inner)[index_con]

    dual = [-w_i; 1 / t_i * w_i * u_i]

    return dual
end

## Reduced costs.
function MOI.get(
    model::Optimizer,
    cd::MOI.ConstraintDual,
    ci::MOI.ConstraintIndex{MOI.VariableIndex,MOI.LessThan{Float64}},
)
    if model.number_solved == 0
        error("ConstraintDual not available.")
    elseif cd.result_index > 1
        throw(MOI.ResultIndexBoundsError{MOI.ConstraintDual}(cd, 1))
    end
    vi = MOI.VariableIndex(ci.value)
    check_inbounds(model, vi)

    # Constraints' duals are before reduced costs in KNITRO.
    offset = number_constraints(model)
    lambda = sense_dual(model) * get_dual(model.inner, vi.value + offset)
    if lambda < 0
        return lambda
    else
        return 0
    end
end

function MOI.get(
    model::Optimizer,
    cd::MOI.ConstraintDual,
    ci::MOI.ConstraintIndex{MOI.VariableIndex,MOI.GreaterThan{Float64}},
)
    if model.number_solved == 0
        error("ConstraintDual not available.")
    elseif cd.result_index > 1
        throw(MOI.ResultIndexBoundsError{MOI.ConstraintDual}(cd, 1))
    end
    vi = MOI.VariableIndex(ci.value)
    check_inbounds(model, vi)

    # Constraints' duals are before reduced costs in KNITRO.
    offset = number_constraints(model)
    lambda = sense_dual(model) * get_dual(model.inner, vi.value + offset)
    if lambda > 0
        return lambda
    else
        return 0
    end
end

function MOI.get(
    model::Optimizer,
    cd::MOI.ConstraintDual,
    ci::MOI.ConstraintIndex{MOI.VariableIndex,MOI.EqualTo{Float64}},
)
    if model.number_solved == 0
        error("ConstraintDual not available.")
    elseif cd.result_index > 1
        throw(MOI.ResultIndexBoundsError{MOI.ConstraintDual}(cd, 1))
    end
    vi = MOI.VariableIndex(ci.value)
    check_inbounds(model, vi)

    # Constraints' duals are before reduced costs in KNITRO.
    offset = number_constraints(model)
    lambda = get_dual(model.inner, vi.value + offset)
    return sense_dual(model) * lambda
end

function MOI.get(model::Optimizer, ::MOI.NLPBlockDual)
    if model.number_solved == 0
        error("NLPBlockDual not available.")
    end
    # Get first index corresponding to a non-linear constraint:
    lambda = get_dual(model.inner)
    # FIXME: Assume that lambda has same sense as for linear
    # and quadratic constraint, but this is not tested inside MOI.
    return sense_dual(model) .* [lambda[i+1] for i in model.nlp_index_cons]
end

###
if KNITRO_VERSION >= v"12.0"
    MOI.get(model::Optimizer, ::MOI.SolveTimeSec) = KN_get_solve_time_cpu(model.inner)
end
# Additional getters
MOI.get(model::Optimizer, ::MOI.NodeCount) = KN_get_mip_number_nodes(model.inner)
MOI.get(model::Optimizer, ::MOI.BarrierIterations) = KN_get_number_iters(model.inner)
MOI.get(model::Optimizer, ::MOI.RelativeGap) = KN_get_mip_rel_gap(model.inner)
MOI.get(model::Optimizer, ::MOI.ObjectiveBound) = KN_get_mip_relaxation_bnd(model.inner)
