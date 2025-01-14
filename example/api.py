if __name__ == "__main__":
    import time
    import numpy as np
    from tcp import TCPConnection

    tcp_conn_client = TCPConnection(port=5000)
    tcp_conn_client.start_client()
    get_joints_l = lambda: tcp_conn_client.send_message("ArmL", 103, {})
    get_joints_r = lambda: tcp_conn_client.send_message("ArmR", 103, {})

    print(get_joints_l()["payload"]["Joints"])
