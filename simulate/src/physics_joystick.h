#pragma once

#include <iostream>
#include <array>
#include <functional>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include "joystick/joystick.h"
#include <memory>

using JoystickAxisFilter =
    std::function<std::array<float, 3>(float lx, float ly, float rx)>;

class XBoxJoystick : public unitree::common::UnitreeJoystick
{
public:
    XBoxJoystick(std::string device, int bits = 15,
                 JoystickAxisFilter axis_filter = {})
	: unitree::common::UnitreeJoystick()
	, axis_filter_(std::move(axis_filter))
	{
		js_ = std::make_unique<Joystick>(device);
		if(!js_->isFound()) {
			std::cout << "Error: Joystick open failed." << std::endl;
			exit(1);
		}
        max_value_ = 1 << (bits - 1);
		if (axis_filter_) {
			// Do not low-pass blend a newly safe command with an older command.
			lx.smooth = 1.0f;
			ly.smooth = 1.0f;
			rx.smooth = 1.0f;
		}
	}

    void update() override
    {
        js_->getState();
        back(js_->button_[6]);
        start(js_->button_[7]);
        LB(js_->button_[4]);
        RB(js_->button_[5]);
        A(js_->button_[0]);
        B(js_->button_[1]); 
        X(js_->button_[2]);
        Y(js_->button_[3]);
        up(js_->axis_[7] < 0);
        down(js_->axis_[7] > 0);
        left(js_->axis_[6] < 0);
        right(js_->axis_[6] > 0);
        LT(js_->axis_[2] > 0);
        RT(js_->axis_[5] > 0);
        std::array<float, 3> axes = {
            static_cast<float>(double(js_->axis_[0]) / max_value_),
            static_cast<float>(-double(js_->axis_[1]) / max_value_),
            static_cast<float>(double(js_->axis_[3]) / max_value_)};
        if (axis_filter_) axes = axis_filter_(axes[0], axes[1], axes[2]);
        lx(axes[0]);
        ly(axes[1]);
        rx(axes[2]);
        ry(-double(js_->axis_[4]) / max_value_);
    }
private:
	std::unique_ptr<Joystick> js_;
	int max_value_;
	JoystickAxisFilter axis_filter_;
};


class SwitchJoystick : public unitree::common::UnitreeJoystick
{
public:
    SwitchJoystick(std::string device, int bits = 15,
                   JoystickAxisFilter axis_filter = {})
	: unitree::common::UnitreeJoystick()
	, axis_filter_(std::move(axis_filter))
	{
		js_ = std::make_unique<Joystick>(device);
		if(!js_->isFound()) {
			std::cout << "Error: Joystick open failed." << std::endl;
			exit(1);
		}
        max_value_ = 1 << (bits - 1);
		if (axis_filter_) {
			lx.smooth = 1.0f;
			ly.smooth = 1.0f;
			rx.smooth = 1.0f;
		}
	}

    void update() override
    {
        js_->getState();
        back(js_->button_[10]);
        start(js_->button_[11]);
        LB(js_->button_[6]);
        RB(js_->button_[7]);
        A(js_->button_[0]);
        B(js_->button_[1]); 
        X(js_->button_[3]);
        Y(js_->button_[4]);
        up(js_->axis_[7] < 0);
        down(js_->axis_[7] > 0);
        left(js_->axis_[6] < 0);
        right(js_->axis_[6] > 0);
        LT(js_->axis_[5] > 0);
        RT(js_->axis_[4] > 0);
        std::array<float, 3> axes = {
            static_cast<float>(double(js_->axis_[0]) / max_value_),
            static_cast<float>(-double(js_->axis_[1]) / max_value_),
            static_cast<float>(double(js_->axis_[2]) / max_value_)};
        if (axis_filter_) axes = axis_filter_(axes[0], axes[1], axes[2]);
        lx(axes[0]);
        ly(axes[1]);
        rx(axes[2]);
        ry(-double(js_->axis_[3]) / max_value_);
    }
private:
	std::unique_ptr<Joystick> js_;
	int max_value_;
	JoystickAxisFilter axis_filter_;
};
