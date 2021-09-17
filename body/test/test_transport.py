# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.transport
import stretch_body.transport.cobs_encoder

import time


class TestTransport(unittest.TestCase):

    # def test_generate_rpc_data(self):
    #     """Test that is_moving_filtered is False when no motion
    #     """
    #     pass
    #     rpc_struct = """
    #     struct __attribute__ ((packed)) Status{
    #       uint8_t mode;                 //current control mode
    #       float effort;                 //ticks, 1 tick = 12.95mA
    #       double pos;                   //rad, wrapped
    #       float vel;                    //rad/sec
    #       float err;                    //controller error (inner loop)
    #       uint32_t diag;                //diagnostic codes     
    #       uint64_t timestamp;           //us of time of when encoder was read (since power-on)
    #       uint64_t timestamp_line_sync; //us of time of when status sync was triggered (since power-on)
    #       float debug; 
    #       uint32_t guarded_event;       //counter of guarded events since power-up
    #       float pos_traj;                 //Target of waypoint trajectory
    #     };
    #     """

    def test_get_status(self):
        RPC_GET_STATUS = 3
        RPC_REPLY_STATUS = 4
        reply_struct = """
        struct __attribute__ ((packed)) Status{
            uint8_t mode;   //current control mode
            float effort;   //ticks, 1 tick = 12.95mA
            double pos;      //rad, wrapped
            float vel;      //rad/sec
            float err;       //controller error (inner loop)
            uint32_t diag;       //diagnostic codes
            uint32_t timestamp; //us
            float debug;
            uint32_t guarded_event;
        };
        """
        usb = '/dev/hello-motor-lift'
        stretch_body.transport.startup(usb)
        reply_dict = stretch_body.transport.rpc.send(
            usb=usb,
            call_type=RPC_GET_STATUS,
            reply_type=RPC_REPLY_STATUS,
            call_struct=None,
            reply_struct=reply_struct,
            call_dict=None
        )