#ifndef FRANKACONTROL_FRANKAMODEL_HPP
#define FRANKACONTROL_FRANKAMODEL_HPP

#include <franka/model.h>

class FrankaModel {
public:
    FrankaModel() = default;
    FrankaModel(const franka::Model& model) : _model(model){};
    virtual ~FrankaModel() = default;

protected:
    franka::Model _model;
};

#endif // FRANKACONTROL_FRANKAMODEL_HPP