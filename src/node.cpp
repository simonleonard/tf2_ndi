#include <ndi_msgs/msg/rigid_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Broadcaster : public rclcpp::Node {

private:

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Subscription<ndi_msgs::msg::RigidArray>::SharedPtr ndi_subscriber;

  std::string reference_frame;
  
public:

  Broadcaster( const std::string& name ) : Node( name ){
    
    ndi_subscriber =
      create_subscription<ndi_msgs::msg::RigidArray>
      ("rigid_poses", 10, std::bind(&Broadcaster::callback, this, std::placeholders::_1 ) );
    tf_broadcaster=std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // param
    declare_parameter( "reference_frame", rclcpp::ParameterType::PARAMETER_STRING );
    if( !get_parameter( "reference_frame", reference_frame ) ){
      RCLCPP_ERROR( get_logger(), "Parameter reference_frame is not set.");
    }
    
  }

  void callback( const ndi_msgs::msg::RigidArray& ndi ){

    if( reference_frame.empty() ){
      RCLCPP_ERROR( get_logger(), "Parameter reference_frame is not set.");
      return;
    }
    
    // scan for ref
    std::vector<std::string>::const_iterator ref_it;
    ref_it = std::find( ndi.frames.begin(), ndi.frames.end(), reference_frame );
    if( ref_it == ndi.frames.end() ){
      RCLCPP_ERROR_STREAM( get_logger(), "Reference frame " << reference_frame
			   << " was not found in the NDI message." );
      return;
    }

    // get the index of the reference frame
    std::size_t ref_idx = ref_it - ndi.frames.begin();
    if( !ndi.inbound[ref_idx] ){
      RCLCPP_ERROR_STREAM( get_logger(), "Reference frame " << reference_frame
                           << " is out of bound. Skipping message." );
      return;
    }
    
    tf2::Transform Rt_ref;
    tf2::fromMsg( ndi.poses[ref_idx], Rt_ref );
    Rt_ref = Rt_ref.inverse();
    
    std::vector<geometry_msgs::msg::TransformStamped> msg;
    // copy frame but change the reference.
    std_msgs::msg::Header header = ndi.header;
    header.frame_id = reference_frame;
    

    for( std::size_t i=0; i<ndi.poses.size(); i++ ){

      if( i != ref_idx ){ // skip ref frame
      
	tf2::Transform Rt;
	tf2::fromMsg( ndi.poses[i], Rt );
	
	Rt = Rt_ref * Rt;
	
	geometry_msgs::msg::TransformStamped msgRt;
	msgRt.header = header;
	msgRt.child_frame_id = ndi.frames[i];
	msgRt.transform = tf2::toMsg( Rt );
	
	msg.push_back( msgRt );

      }
    }

    tf_broadcaster->sendTransform(msg);
    
  }
  
};

int main(int argc, char** argv){

  rclcpp::init( argc, argv );
  std::shared_ptr<Broadcaster> broadcaster = std::make_shared<Broadcaster>("tf2_broadcaster");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(broadcaster);

  executor.spin();
  rclcpp::shutdown();
  
  return 0;

}
