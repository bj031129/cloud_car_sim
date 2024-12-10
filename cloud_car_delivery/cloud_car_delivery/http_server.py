from flask import Flask, Response, request
import json
import logging


app = Flask(__name__)

# 关闭 Flask 的默认日志
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# 创建一个全局变量来存储图像数据
current_image = None


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
        return Response(
            json.dumps({
                'data': {
                    'delivery_address': 'TM-3-19',
                    'status': 'busy'
                },
                'code': 50021,
                'msg': '状态接收成功'
            }),
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


def main():
    app.run(host='0.0.0.0', port=8000)


if __name__ == '__main__':
    main()