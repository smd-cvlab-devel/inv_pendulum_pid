#!/usr/bin/env python

PACKAGE='inv_pendulum_pid'

from driver_base.msg import SensorLevels
from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator( )

gen.add( "p_gain", double_t, SensorLevels.RECONFIGURE_RUNNING, "Proportional Gain", 0.0, 0.0, 100.0 )
gen.add( "i_gain", double_t, SensorLevels.RECONFIGURE_RUNNING, "Integral Gain", 0.0, 0.0, 255.0 )
gen.add( "d_gain", double_t, SensorLevels.RECONFIGURE_RUNNING, "Derivative Gain", 0.0, 0.0, 10000.0 )

exit( gen.generate( PACKAGE, "inv_pendulum_pid", "InvPendulumPID" ) )

