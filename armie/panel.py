"""Local desktop operator panel; no HMI display or face graphics."""
import json
from .common import STATE_QOS


def main():
    import tkinter as tk
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init()
    node=Node('armie_panel')
    root=tk.Tk()
    root.title('Armie · 桌面宠物')
    root.geometry('640x440')
    label=tk.StringVar(value='等待 Armie 状态…')
    info=tk.StringVar(value='数字手势或按钮选择玩法')
    status_at=[0.]
    import time
    def state(msg):
        value=json.loads(msg.data)
        if value.get('schema_version')!=2:return
        status_at[0]=time.monotonic()
        label.set(f'{"模拟演示" if value["mock"] else "实机"} | {value["mode"] or "待机"} | {value["phase"]}')
        info.set(value['message']+'\n'+(value['outcome'] or ''))
    sub=node.create_subscription(String,'/armie/state',state,STATE_QOS)
    pub=node.create_publisher(String,'/armie/command',10)
    clients={name:node.create_client(Trigger,'/armie/'+name) for name in ['finish','release','stop','ready','record_front','record_table']}
    def send(command):
        if command in clients:
            client=clients[command]
            if not client.service_is_ready():info.set('服务未就绪');return
            future=client.call_async(Trigger.Request())
            def done(f):
                try:info.set(f.result().message)
                except Exception:info.set('请求未完成，请查看状态')
            future.add_done_callback(done)
        else:pub.publish(String(data=command))
    tk.Label(root,text='Armie',font=('',24)).pack(pady=12)
    tk.Label(root,textvariable=label,font=('',13)).pack()
    tk.Label(root,textvariable=info,wraplength=600,height=3).pack()
    modes=tk.Frame(root);modes.pack(pady=10)
    for i,text in enumerate(['1 · 软软模式','2 · 好奇小球','3 · 舞蹈','4 · 看着我']):
        tk.Button(modes,text=text,width=14,height=2,command=lambda n=i+1:send(str(n))).grid(row=0,column=i,padx=3)
    actions=tk.Frame(root);actions.pack(pady=12)
    for i,(name,text) in enumerate([('finish','结束并回正'),('release','确认已松手'),('stop','暂停（不回正）')]):
        tk.Button(actions,text=text,width=17,height=2,command=lambda n=name:send(n)).grid(row=0,column=i,padx=3)
    setup=tk.Frame(root);setup.pack()
    for i,(name,text) in enumerate([('ready','准备 / 恢复'),('record_front','记录朝人姿态'),('record_table','记录桌面观察位')]):
        tk.Button(setup,text=text,width=17,command=lambda n=name:send(n)).grid(row=0,column=i,padx=3)
    tk.Label(root,text='暂停按钮仅取消本程序的动作，不代替硬件急停。').pack(pady=15)
    def poll():
        if rclpy.ok():rclpy.spin_once(node,timeout_sec=0.)
        if status_at[0] and time.monotonic()-status_at[0]>2.:label.set('连接中断：请检查游戏节点')
        root.after(25,poll)
    def close():
        send('stop')
        until=time.monotonic()+.3
        while rclpy.ok() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        root.destroy()
    root.protocol('WM_DELETE_WINDOW',close)
    root.after(25,poll)
    try:root.mainloop()
    finally:node.destroy_node();rclpy.try_shutdown()
