#include "inv_pendulum_pid/inv_pendulum_pid.hpp"

#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <iomanip>

PLUGINLIB_DECLARE_CLASS(inv_pendulum_pid, InvPendulumPID, inv_pendulum_pid::InvPendulumPID, nodelet::Nodelet)

namespace inv_pendulum_pid
{
	InvPendulumPID::InvPendulumPID( ) :
		p( 0.0 ),
		i( 0.0 ),
		d( 0.0 ),
		have_uav( false ),
		have_pendulum( false ),
		last_theta( 0.0 ),
		last_phi( 0.0 )
	{
	}

	void InvPendulumPID::onInit( )
	{
		nh = getNodeHandle( );
		nh_priv = getPrivateNodeHandle( );
		nh_priv.param( "p", p, 0.0 );
		nh_priv.param( "i", i, 0.0 );
		nh_priv.param( "d", d, 0.0 );
		dyn_re = new dynamic_reconfigure::Server<inv_pendulum_pid::InvPendulumPIDConfig>( nh_priv );
		dyn_re_cb_type = new dynamic_reconfigure::Server<inv_pendulum_pid::InvPendulumPIDConfig>::CallbackType( boost::bind( &InvPendulumPID::DynReCB, this, _1, _2) );
		dyn_re->setCallback( *dyn_re_cb_type );
		twist_pub = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1, false );
		uav_odom_sub = nh.subscribe( "uav_odom", 1, &InvPendulumPID::UAVOdomCB, this );
		pendulum_odom_sub = nh.subscribe( "pendulum_odom", 1, &InvPendulumPID::PendulumOdomCB, this );
	}

	void InvPendulumPID::UAVOdomCB( const nav_msgs::OdometryPtr &msg )
	{
		uav_odom = msg;
		have_uav = true;
		if( have_pendulum )
		{
			Iterate( );
			have_uav = false;
			have_pendulum = false;
		}
	}

	void InvPendulumPID::PendulumOdomCB( const nav_msgs::OdometryPtr &msg )
	{
		pendulum_odom = msg;
		have_pendulum = true;
		if( have_uav )
		{
			Iterate( );
			have_uav = false;
			have_pendulum = false;
		}
	}

	void InvPendulumPID::DynReCB( inv_pendulum_pid::InvPendulumPIDConfig &cfg, const uint32_t lvl )
	{
		p = cfg.p_gain;
		i = cfg.i_gain;
		d = cfg.d_gain;
	}

	void InvPendulumPID::Iterate( )
	{
		//
		// GET DATA
		//

		// Get the tf::Quaternions
		const tf::Quaternion uav_quat( uav_odom->pose.pose.orientation.x, uav_odom->pose.pose.orientation.y,
			uav_odom->pose.pose.orientation.z, uav_odom->pose.pose.orientation.w );
		const tf::Quaternion pendulum_quat( pendulum_odom->pose.pose.orientation.x, pendulum_odom->pose.pose.orientation.y,
			pendulum_odom->pose.pose.orientation.z, pendulum_odom->pose.pose.orientation.w );

		const double w = pendulum_quat.w( );
		const double x = pendulum_quat.x( );
		const double y = pendulum_quat.y( );
		const double z = pendulum_quat.z( );

		const double theta = atan2(2.0 * (y*z + w*x), w*w - x*x - y*y + z*z);
		const double phi = asin(-2.0 * (x*z - w*y));

		const double theta_dot = pendulum_odom->twist.twist.angular.x;
		const double phi_dot = pendulum_odom->twist.twist.angular.y;

		//
		// CONTROLLER
		//
		const double vx = p * phi + d * 100 * ( phi - last_phi );
		const double vy = -p * theta - d * 100 * ( theta - last_theta );

		//
		// CONSTRUCT TWIST MESSAGE
		//

		geometry_msgs::TwistPtr msg( new geometry_msgs::Twist );

		msg->linear.x = vx;
		msg->linear.y = vy;

		if( msg->linear.x > 1.0 )
			msg->linear.x = 1.0;
		else if( msg->linear.x < -1.0 )
			msg->linear.x = -1.0;
		if( msg->linear.y > 1.0 )
			msg->linear.y = 1.0;
		else if( msg->linear.y < -1.0 )
			msg->linear.y = -1.0;

		twist_pub.publish( msg );

		last_theta = theta;
		last_phi = phi;
	}
}
