const gui_host = '127.0.0.1'
const gui_port = '8080'

let ws;
setup_ws();

function setup_ws() {
    ws = new WebSocket('ws://' + gui_host + ':' + gui_port + '/solver');
    ws.onopen = () => {
        console.log('Connected to "ws://' + gui_host + ':' + gui_port + '/solver"');
    };
    ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.type) {
            case 'show_face':
                console.log('Setting robot face to: ' + c_msg.facial_expression);

                document.getElementById('robot_face').src = 'static/faces/' + c_msg.facial_expression + '.gif';
                document.getElementById('robot_face').height = 700
                break;
        }
    }
    ws.onclose = () => setTimeout(setup_ws, 1000);
}