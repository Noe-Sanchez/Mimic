#include <mavlink.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>

// Union for handling PX4 custom modes (solo pa mamar)
union px4_custom_mode {
  struct {
    uint16_t reserved;
    uint8_t main_mode;
    uint8_t sub_mode;
  };
  uint32_t data;
  float data_float;
  struct {
    uint16_t reserved_hl;
    uint16_t custom_mode_hl;
  };
};

// Function prototypes
void* send_heartbeat( void *ptr );
void* send_gps( void *ptr );

// Global variables for message and socket
mavlink_message_t msg;
mavlink_status_t status;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;
int sock;
struct sockaddr_in addr;

int main(int argc, char *argv[]) {
  // Open general UDP socket, simulator only
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(14550);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // Use loopback address, change for remote

  // Confirm socket opened
  if(sock < 0){ printf("Failed to open socket\n"); return 1; }

  // Create threads
  pthread_t heartbeat_thread;
  //pthread_create(&heartbeat_thread, NULL, send_heartbeat, NULL);
  pthread_create(&heartbeat_thread, NULL, &send_heartbeat, NULL); 
  pthread_t gps_thread;
  //pthread_create(&gps_thread, NULL, send_gps, NULL);
  pthread_create(&gps_thread, NULL, &send_gps, NULL);

  // Message and command handling in main thread
  int bytes_read;
  int handled_message;
  while (1) {
    bytes_read = recvfrom(sock, buf, MAVLINK_MAX_PACKET_LEN, 0, NULL, NULL);
    for (uint16_t i = 0; i < bytes_read; i++) { 
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
	printf("Received message with ID %d\n", msg.msgid);
	//Handle message ID
	if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
	  handled_message = 1;
	  mavlink_command_long_t command_long;
	  mavlink_msg_command_long_decode(&msg, &command_long);
	  printf("Received command with command ID %d\n", command_long.command); 
	  if (command_long.command == MAV_CMD_REQUEST_MESSAGE) {
	    // Print received message id
	    printf("Received request message of ID %f\n", command_long.param1);
	    //Check if PROTOCOL_VERSION is requested
	    if (command_long.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
	      printf("Received request autopilot version\n");
	      // Respond with AUTOPILOT_VERSION message
	      //const uint8_t * version_bytes = "DanSFirmware";
	      const uint8_t * version_bytes = "5b85859";
	      const uint8_t * uid2 = "DanS"; 
	      mavlink_msg_autopilot_version_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, (uint64_t)8223, 2, 2, 2, 52, version_bytes, version_bytes, version_bytes, 45, 1, 1, uid2);
	      len = mavlink_msg_to_send_buffer(buf, &msg);
	      sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	      handled_message = 0;
	    }else if (command_long.param1 == MAVLINK_MSG_ID_PROTOCOL_VERSION) {
	      printf("Received request protocol version\n");
	      // Respond with PROTOCOL_VERSION message
	      mavlink_msg_protocol_version_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, 2, 2, 2, "DanS", "DanS");
	      len = mavlink_msg_to_send_buffer(buf, &msg);
	      sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	      handled_message = 0;
	    }
	  }else if (command_long.param1 == MAV_CMD_PREFLIGHT_CALIBRATION) { 
	    printf("We received request for preflight calibration\n");
	    handled_message = 0;
	  }
	  if (handled_message == 0) {
	    // Send command ack
	    mavlink_msg_command_ack_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, command_long.command, MAV_RESULT_ACCEPTED, 100, 0, 1, 1);
	    len = mavlink_msg_to_send_buffer(buf, &msg);
	    sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	    printf("Acknowledged command %d\n", command_long.command);
	  }else{
	    // Send command ack
	    //mavlink_msg_command_ack_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, command_long.command, MAV_RESULT_DENIED, 100, 0, 1, 1); 
	    mavlink_msg_command_ack_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, command_long.command, MAV_RESULT_UNSUPPORTED, 100, 0, 1, 1); 
	    len = mavlink_msg_to_send_buffer(buf, &msg);
	    sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	    printf("Denied command %d\n", command_long.command);
	  }
	// Dont print heartbeat messages for now
	//}else if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
	//  printf("Received QGC heartbeat\n");
	}else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
	  printf("Received command int\n");
	  // TODO: Implement command int handling
	}else if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
	  int param_index = 0;
	  int param_count = 5;
	  printf("Received parameter request list\n");
	  // Send empty parameter value
	  mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "test", 0, MAV_PARAM_TYPE_INT32, param_index, param_count);
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	  param_index++;
	  // Send SYS_AUTOSTART parameter value of 4011
	  //mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "SYS_AUTOSTART", 4001, MAV_PARAM_TYPE_REAL32, 0, 0); 
	  mavlink_param_union_t param_value;
	  param_value.param_int32 = 4001;
	  mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "SYS_AUTOSTART", param_value.param_float, MAV_PARAM_TYPE_INT32, param_index, param_count);
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	  param_index++;
	  param_value.param_int32 = 1;
	  mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "MAV_SYS_ID", param_value.param_float, MAV_PARAM_TYPE_INT32, param_index, param_count);
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	  param_index++;
	  param_value.param_int32 = 0;
	  mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "SYS_AUTOCONFIG", param_value.param_float, MAV_PARAM_TYPE_INT32, param_index, param_count);
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	  param_index++;
	  param_value.param_int32 = 0;
	  mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "COM_RC_IN_MODE", param_value.param_float, MAV_PARAM_TYPE_INT32, param_index, param_count);
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	}else if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST) {
	  printf("Received mission request list\n");
	  // Send empty mission count
	  mavlink_msg_mission_count_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, 1, MAV_COMP_ID_AUTOPILOT1, 0, MAV_MISSION_TYPE_MISSION, 1);
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	}else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
	  printf("GCS acknowledged mission\n");
	}
      }
    } 
  }
  
  return 0;
}

// Thread definitions

void* send_heartbeat( void *ptr ) {
  union px4_custom_mode custom_mode;
  while(1) {
    custom_mode.data = 0;
    custom_mode.main_mode = 1;
    custom_mode.sub_mode = 0;
    mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 13, custom_mode.data, MAV_STATE_STANDBY); 
    //mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, 12, 0, MAV_STATE_STANDBY); 
    len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
    usleep(250000);
  }
}

void* send_gps( void *ptr ) {
  // Send global origin
  mavlink_msg_gps_global_origin_pack_chan(1, 1, MAVLINK_COMM_0, &msg, 25.649033781863928 * 1E7, -100.28982120113689 * 1E7, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
  while(1) {
    // Use global position int pack chan
    mavlink_msg_global_position_int_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, 0, 25.649033781863928 * 1E7, -100.28982120113689 * 1E7, 0, 0, 0, 0, 0, 0);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
    usleep(1000000);
  }
}
