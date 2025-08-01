#include <ndi_msgs/msg/rigid_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Publisher : public rclcpp::Node {

private:

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_publisher;
  rclcpp::Subscription<ndi_msgs::msg::RigidArray>::SharedPtr ndi_subscriber;

  std::string reference_frame;
  std::string tracked_frame;
  
public:

  Publisher( const std::string& name ) : Node( name ){
    
    ndi_subscriber =
      create_subscription<ndi_msgs::msg::RigidArray>
      ("rigid_poses", 10, std::bind(&Publisher::callback, this, std::placeholders::_1 ) );
    point_publisher = create_publisher<geometry_msgs::msg::Point>("/tracked_tip", 10);

    // param
    declare_parameter( "reference_frame", rclcpp::ParameterType::PARAMETER_STRING );
    if( !get_parameter( "reference_frame", reference_frame ) ){
      RCLCPP_ERROR( get_logger(), "Parameter reference_frame is not set.");
    }
    declare_parameter( "tracked_frame", rclcpp::ParameterType::PARAMETER_STRING );
    if( !get_parameter( "tracked_frame", tracked_frame ) ){
      RCLCPP_ERROR( get_logger(), "Parameter tracked_frame is not set.");
    }
    
  }

  void callback( const ndi_msgs::msg::RigidArray& ndi ){

    if( reference_frame.empty() ){
      RCLCPP_ERROR( get_logger(), "Parameter reference_frame is not set.");
      return;
    }
    if( tracked_frame.empty() ){
      RCLCPP_ERROR( get_logger(), "Parameter tracked_frame is not set.");
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

    // scan for trk
    std::vector<std::string>::const_iterator trk_it;
    trk_it = std::find( ndi.frames.begin(), ndi.frames.end(), tracked_frame );
    if( trk_it == ndi.frames.end() ){
      RCLCPP_ERROR_STREAM( get_logger(), "Tracked frame " << tracked_frame
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
    
    // get the index of the tracked frame
    std::size_t trk_idx = trk_it - ndi.frames.begin();
    if( !ndi.inbound[trk_idx] ){
      RCLCPP_ERROR_STREAM( get_logger(), "Tracked frame " << tracked_frame
                           << " is out of bound. Skipping message." );
      return;
    }

    // get and inverse the reference
    tf2::Transform Rt_ref;
    tf2::fromMsg( ndi.poses[ref_idx], Rt_ref );
    Rt_ref = Rt_ref.inverse();


    // get tracked
    tf2::Transform Rt_trk;
    tf2::fromMsg( ndi.poses[trk_idx], Rt_trk );
    
    // copy frame but change the reference. Not using.
    std_msgs::msg::Header header = ndi.header;
    header.frame_id = reference_frame;
    

    Rt_trk = Rt_ref * Rt_trk;

    geometry_msgs::msg::Point xyz;
    tf2::toMsg( Rt_trk.getOrigin(), xyz );
    /*
    geometry_msgs::msg::TransformStamped msgRt;
    msgRt.header = header;
    msgRt.child_frame_id = ndi.frames[i];
    msgRt.transform = tf2::toMsg( Rt );
    */

    point_publisher->publish( xyz );
    
  }
  
};

int main(int argc, char** argv){

  rclcpp::init( argc, argv );
  std::shared_ptr<Publisher> publisher = std::make_shared<Publisher>("point_publisher");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(publisher);

  executor.spin();
  rclcpp::shutdown();
  
  return 0;

}
