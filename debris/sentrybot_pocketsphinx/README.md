To use keyword spotting mode and send the robot autonomously by saying the 6 triggered keywords locations:
  1. Launch the kws.launch file by giving it 3 arguments, the path of, .dic (dict:=), .kwlist (kws:=), language_model_folder (hmm:=) (src/demo/cmusphinx-en-us-8khz-5.2).
  2. The python file set_goal_custom.py located in scripts can be run to send goal to move_base node.
