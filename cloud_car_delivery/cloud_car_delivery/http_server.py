from flask import Flask, Response, request # type: ignore
import json
import logging


app = Flask(__name__)

# 关闭 Flask 的默认日志
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# 创建一个全局变量来存储图像数据
current_image = None

# 添加全局变量跟踪取货状态
pickup_status = False


@app.route('/robot/video_stream')
def video_stream():
    return Response(generate(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')


def generate():
    while True:
        if current_image is not None:
            yield (b'--frame\r\n'
                  b'Content-Type: image/jpeg\r\n\r\n' + current_image + b'\r\n')
        else:
            yield (b'--frame\r\n'
                  b'Content-Type: text/plain\r\n\r\nNo image available\r\n')
            print("发送图像失败")


@app.route('/robot/video', methods=['POST'])
def get_image():
    global current_image
    try:
        current_image = request.data
        # print("图像接收成功，数据大小:", len(current_image))

        return Response(
            json.dumps({
                'data': None,
                'code': 50011,
                'msg': '图像接收成功'
            }),
            status=200,
            mimetype='application/json'
        )
    except Exception as e:
        print("处理图像时出错:", str(e))
        return Response(
            json.dumps({
                'data': None,
                'code': 50012,
                'msg': str(e)
            }),
            status=500,
            mimetype='application/json'
        )


@app.route('/robot/state', methods=['POST'])
def get_state():
    """
    响应配送请求的端点，返回配送地址和状态
    """
    try:
        data = request.json
        print(f"收到机器人{data['robotId']}的状态请求: "
              f"经度={data['longitude']}, "
              f"纬度={data['latitude']}, "
              f"电量={data['battery']}"
        )
        # 构建响应数据
        response_data = {
            'data': None,
            'code': 50021,
            'msg': '状态接收成功'
        }

        # 如果机器人状态为空闲,添加配送地址
        if data['status'] == 'idle':
            print("机器人状态为空闲,添加配送地址")
            response_data['data'] = {
                'delivery_address': 'TM-2-4'
            }
        else:
            response_data['data'] = {
                'delivery_address': None
            }

        return Response(
            json.dumps(response_data),
            status=200,
            mimetype='application/json'
        )
    except Exception as e:
        return Response(
            json.dumps({
                'data': None,
                'code': 50022,
                'msg': str(e)
            }),
            status=500,
            mimetype='application/json'
        )


@app.route('/robot/arrived', methods=['POST'])
def get_arrived():
    """
    处理机器人到达信息的端点
    """
    try:
        data = request.json
        print(f"收到机器人{data['robotId']}的到达信息")
        return Response(
            json.dumps({
                'data': None,
                'code': 50031,
                'msg': '到达信息接收成功'
            }),
            status=200,
            mimetype='application/json'
        )
    except Exception as e:
        return Response(
            json.dumps({
                'data': None,
                'code': 50032,
                'msg': str(e)
            }),
            status=500,
            mimetype='application/json'
        )


@app.route('/robot/pickup', methods=['POST'])
def check_pickup():
    """
    检查用户是否已取货的端点
    """
    global pickup_status
    try:
        data = request.json
        robot_id = data.get('robotId')
        print(f"收到机器人{robot_id}的取货状态查询")
        # 这里应该是检查实际的取货状态，现在用全局变量模拟
        # 在实际应用中，可能需要查询数据库或其他服务
        pickup_status = True  # 模拟用户已取货
        
        return Response(
            json.dumps({
                'data': {
                    'isPicked': pickup_status
                },
                'code': 50041,
                'msg': '取货状态查询成功'
            }),
            status=200,
            mimetype='application/json'
        )
    except Exception as e:
        return Response(
            json.dumps({
                'data': None,
                'code': 50042,
                'msg': str(e)
            }),
            status=500,
            mimetype='application/json'
        )


@app.route('/robot/back', methods=['POST'])
def get_back():
    """
    处理机器人回到站点的信息
    """
    try:
        data = request.json
        print(f"收到机器人{data['robotId']}的回站信息")
        return Response(
            json.dumps({
                'data': None,
                'code': 50051,
                'msg': '回站信息接收成功'
            }),
            status=200,
            mimetype='application/json'
        )
    except Exception as e:
        return Response(
            json.dumps({
                'data': None,
                'code': 50052,
                'msg': str(e)
            }),
            status=500,
            mimetype='application/json'
        )


@app.route('/robot/velocity', methods=['POST'])
def get_vel():
    """
    处理机器人速度信息的端点
    """
    try:
        data = request.json
        print(f"收到机器人{data['robotId']}的速度信息: "
              f"线速度={data['linear_velocity']*10:.4f}, "
              f"角速度={data['angular_velocity']:.4f}")
        return Response(
            json.dumps({
                'data': None,
                'code': 50061,
                'msg': '速度信息接收成功'
            }),
            status=200,
            mimetype='application/json'
        )
    except Exception as e:
        return Response(
            json.dumps({
                'data': None,
                'code': 50062,
                'msg': str(e)
            }),
            status=500,
            mimetype='application/json'
        )
    

def main():
    app.run(host='0.0.0.0', port=8000)


if __name__ == '__main__':
    main()

# ros2 topic pub /maintenance std_msgs/msg/String "data: '电机'"