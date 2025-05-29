#pragma once

#include "control/state/state.hpp"
#include "control/model/control_input.hpp"

namespace control::model
{
    class ModelBase
    {
    public:
        virtual ~ModelBase() = default;

        virtual std::unique_ptr<control::state::State> f(const control::state::State &state, const control::model::Input &input) const = 0;
    };

} // namespace control::model
