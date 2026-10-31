import json
import os
import signal
import subprocess
import textwrap
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from armie.common import STATE_QOS
from test_ros import until


def test_installed_game_launch_and_real_panel_buttons(tmp_path):
    env=dict(os.environ,ROS_DOMAIN_ID='188',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',MPLCONFIGDIR='/tmp/armie-mpl')
    config=tmp_path/'game.yaml'
    config.write_text('armie_game:\n  ros__parameters:\n    mock_step_sec: 0.03\n    face_duration_sec: 1.0\n')
    logfile=tmp_path/'game-launch.log'
    panel_script=textwrap.dedent('''
        import tkinter as tk
        original=tk.Tk
        def create():
            root=original()
            root.withdraw()
            def invoke(text):
                def search(widget):
                    for child in widget.winfo_children():
                        if isinstance(child,tk.Button) and child.cget('text')==text:
                            child.invoke();return True
                        if search(child):return True
                    return False
                assert search(root),text
            root.after(600,lambda:invoke('1 · 软软模式'))
            root.after(1300,lambda:invoke('结束并回正'))
            root.after(1900,lambda:invoke('确认已松手'))
            root.after(3500,lambda:root.eval(root.protocol('WM_DELETE_WINDOW')))
            return root
        tk.Tk=create
        from armie.panel import main
        main()
    ''')
    with logfile.open('w') as log:
        launcher=subprocess.Popen(['ros2','launch','armie','armie.launch.py',f'config:={config}'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        panel=None
        rclpy.init(domain_id=188)
        node=Node('game_launch_probe');exe=SingleThreadedExecutor();exe.add_node(node)
        states=[]
        sub=node.create_subscription(String,'/armie/state',lambda msg:states.append(json.loads(msg.data)),STATE_QOS)
        try:
            client=node.create_client(Trigger,'/armie/finish')
            until(exe,lambda:states and client.service_is_ready(),timeout=8.)
            assert states[-1]['schema_version']==2
            assert not any(name=='armie_hmi' for name,_ in node.get_node_names_and_namespaces())
            panel=subprocess.Popen(['python3','-c',panel_script],env=env,stdout=log,stderr=subprocess.STDOUT)
            until(exe,lambda:panel.poll() is not None,timeout=8.)
            assert panel.returncode==0,logfile.read_text()
            assert any(s['phase']=='dragging' for s in states)
            assert any(s['phase']=='waiting_release' for s in states)
            assert any(s['state']=='idle' and s['round_id']==1 for s in states)
        finally:
            if panel is not None and panel.poll() is None:panel.terminate();panel.wait(timeout=3.)
            exe.shutdown();node.destroy_node();rclpy.shutdown()
            launcher.send_signal(signal.SIGINT)
            try:launcher.wait(timeout=8.)
            except subprocess.TimeoutExpired:os.killpg(launcher.pid,signal.SIGKILL);launcher.wait(timeout=2.)
        assert launcher.returncode==0,logfile.read_text()
    assert 'Traceback' not in logfile.read_text(),logfile.read_text()
