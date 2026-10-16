
import socket
import json
from flask import Flask, render_template_string, jsonify, request

app = Flask(__name__)
SOCKET_PATH = "/tmp/scada.sock"

def query_socket(command):
    try:
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.settimeout(1.0)
        client.connect(SOCKET_PATH)
        client.sendall((command + "\n").encode('utf-8'))
        response = client.recv(1024).decode('utf-8')
        client.close()
        return response.strip()
    except Exception as e:
        return json.dumps({"p": 0.0, "f": 0.0, "t": 0.0, "valve": "UNKNOWN", "online": False, "error": str(e)})

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Industrial SCADA Dashboard</title>
    <style>
        :root {
            --bg-color: #0f172a;
            --card-bg: #1e293b;
            --accent-primary: #3b82f6;
            --accent-success: #10b981;
            --accent-danger: #ef4444;
            --text-main: #f8fafc;
            --text-muted: #94a3b8;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-main);
            margin: 0;
            padding: 20px;
        }
        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 2px solid #334155;
            padding-bottom: 15px;
            margin-bottom: 25px;
        }
        .header h1 {
            margin: 0;
            font-size: 24px;
            letter-spacing: 1px;
            color: var(--text-main);
        }
        .badge {
            padding: 6px 14px;
            border-radius: 20px;
            font-weight: bold;
            font-size: 14px;
            text-transform: uppercase;
        }
        .badge.online { background-color: var(--accent-success); color: white; }
        .badge.offline { background-color: var(--accent-danger); color: white; }

        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 20px;
            margin-bottom: 25px;
        }
        .card {
            background-color: var(--card-bg);
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.3);
            text-align: center;
            border-top: 4px solid var(--accent-primary);
        }
        .card h3 {
            margin: 0 0 10px 0;
            color: var(--text-muted);
            font-size: 14px;
            text-transform: uppercase;
        }
        .value {
            font-size: 36px;
            font-weight: bold;
            margin: 10px 0;
        }
        .unit {
            font-size: 14px;
            color: var(--text-muted);
        }
        .control-panel {
            background-color: var(--card-bg);
            border-radius: 12px;
            padding: 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.3);
        }
        .btn {
            flex: 1;
            margin: 0 10px;
            padding: 15px;
            font-size: 16px;
            font-weight: bold;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            color: white;
            transition: opacity 0.2s;
        }
        .btn:hover { opacity: 0.9; }
        .btn-open { background-color: var(--accent-success); }
        .btn-close { background-color: var(--accent-danger); }
    </style>
</head>
<body>
    <div class="header">
        <h1>Industrial SCADA Gateway Control</h1>
        <div id="status-badge" class="badge offline">Connecting...</div>
    </div>

    <div class="grid">
        <div class="card" style="border-top-color: #3b82f6;">
            <h3>Pressure</h3>
            <div class="value" id="val-p">0.0</div>
            <div class="unit">bar</div>
        </div>
        <div class="card" style="border-top-color: #8b5cf6;">
            <h3>Flow Rate</h3>
            <div class="value" id="val-f">0.0</div>
            <div class="unit">L/min</div>
        </div>
        <div class="card" style="border-top-color: #f59e0b;">
            <h3>Temperature</h3>
            <div class="value" id="val-t">0.0</div>
            <div class="unit">&deg;C</div>
        </div>
        <div class="card" style="border-top-color: #ec4899;">
            <h3>Valve State</h3>
            <div class="value" id="val-valve" style="font-size: 28px; margin-top: 15px;">---</div>
        </div>
    </div>

    <div class="control-panel">
        <span style="font-weight: bold; color: var(--text-muted);">ACTUATOR OVERRIDE:</span>
        <button class="btn btn-open" onclick="sendCmd('VALVE:OPEN')">OPEN VALVE</button>
        <button class="btn btn-close" onclick="sendCmd('VALVE:CLOSE')">CLOSE VALVE</button>
    </div>

    <script>
        function fetchData() {
            fetch('/api/data')
                .then(res => res.json())
                .then(data => {
                    document.getElementById('val-p').innerText = data.p;
                    document.getElementById('val-f').innerText = data.f;
                    document.getElementById('val-t').innerText = data.t;
                    document.getElementById('val-valve').innerText = data.valve;
                    
                    const badge = document.getElementById('status-badge');
                    if (data.online) {
                        badge.innerText = "ONLINE";
                        badge.className = "badge online";
                    } else {
                        badge.innerText = "OFFLINE";
                        badge.className = "badge offline";
                    }
                })
                .catch(err => console.error("Fetch error:", err));
        }

        function sendCmd(cmd) {
            fetch('/api/cmd', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command: cmd })
            })
            .then(res => res.json())
            .then(data => {
                console.log("Command response:", data);
                fetchData();
            });
        }

        setInterval(fetchData, 1000);
        fetchData();
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/data')
def get_data():
    raw = query_socket("ALL")
    try:
        data = json.loads(raw)
    except:
        data = {"p": 0.0, "f": 0.0, "t": 0.0, "valve": "UNKNOWN", "online": False}
    return jsonify(data)

@app.route('/api/cmd', methods=['POST'])
def post_cmd():
    req = request.get_json()
    cmd = req.get('command', '')
    res = query_socket(cmd)
    return jsonify({"status": "sent", "response": res})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=False)
