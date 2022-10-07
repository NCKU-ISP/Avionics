function rbf_send() {
    socket.send('RBF');
}

function angle_send(angle) {
    socket.send('count' + angle);
}

function init_send() {
    socket.send('init');
}

function stall_send() {
    socket.send('stall');
}

function preLaunch_send() {
    socket.send('preLaunch');
}

function launch_send() {
    socket.send('launch');
}

function abort_send() {
    socket.send('abort');
}

function message_send(message) {
    socket.send(message);
}

function parachute_send() {
    socket.send('open');
}

function emergency_send() {
    socket.send('stop');
}