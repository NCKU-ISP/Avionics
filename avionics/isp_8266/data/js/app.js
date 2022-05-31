if ('serviceWorker' in navigator) {
    navigator.serviceWorker.register('/sw.js')
        .then((reg) => console.log('service worker registered'))
        .catch((err) => console.log('service worker not registered'));
};

var socket;

window.onload = function () {
    socket_connect();
    document.getElementById('body').style.left = 0 + 'px';
    document.getElementById('body').style.top = 0 + 'px';
    console.log("onload");
    move_page();
    button();
    keyin();
    socket_receive();
}

var page = document.getElementById('body');
var windowHeight = window.innerHeight;

function move_page() {
    var page_pos = { x: 0, y: 0 };
    var touch = { x: 0, y: 0 };
    var now = { x: 0, y: 0 };
    var dir = 0;
    page.style.top = '0px';

    page.addEventListener('touchstart', (e) => {
        touch.x = e.targetTouches[0].clientX;
        touch.y = e.targetTouches[0].clientY;
        now.x = parseInt(page.style.left);
        now.y = parseInt(page.style.top);
        page_pos.x = touch.x - now.x;
        page_pos.y = touch.y - now.y;
        textbox.blur();
    })

    page.addEventListener('touchmove', (e) => {
        page.style.transition = '';
        var change = { x: e.targetTouches[0].clientX - touch.x, y: e.targetTouches[0].clientY - touch.y };
        var pos = { x: e.targetTouches[0].clientX - page_pos.x, y: e.targetTouches[0].clientY - page_pos.y };
        if (Math.abs(change.x) > 10 && dir != 2) {
            page.style.left = pos.x + 'px';
            dir = 1;
        } else if (Math.abs(change.y) > 10 && dir != 1) {
            page.style.top = pos.y + 'px';
            dir = 2;
        }
    })

    page.addEventListener('touchend', (e) => {
        var change = { x: e.changedTouches[0].clientX - touch.x, y: e.changedTouches[0].clientY - touch.y };
        var threshould = screen.width / 6;

        if(dir == 1) {
            if (now.x == 0 && change.x > 0 || now.x == -2 * screen.width && change.x < 0) {
                page.style.transition = 'all .2s';
                page.style.left = now.x + 'px';
            } else if (Math.abs(change.x) > threshould) {
                page.style.transition = 'all .3s';
                page.style.left = now.x + (change.x < 0 ? -1 : 1) * screen.width + 'px';
            } else {
                page.style.transition = 'all .2s';
                page.style.left = now.x + 'px';
            }
        } else if(dir == 2) {
            if (now.y == 0 && change.y > 0 || now.y == -1 * window.innerHeight && change.y < 0) {
                if(change.y > 400) window.location.replace('/index.html');
                page.style.transition = 'all .2s';
                page.style.top = now.y + 'px';
            } else if (Math.abs(change.y) > threshould) {
                page.style.transition = 'all .3s';
                page.style.top = now.y + (change.y < 0 ? -1 : 1) * window.innerHeight + 'px';
            } else {
                page.style.transition = 'all .2s';
                page.style.top = now.y + 'px';
            }
        }
        dir = 0;
    })
}

function command_fixed(input)
{
    if(input) {
        document.getElementsByClassName('nav')[3].style.position = 'fixed';
        textbox.style.position = 'fixed';
        send.style.position = 'fixed';
        document.getElementById('console').style.position = 'fixed';
        document.getElementsByClassName('clear-text')[0].style.position = 'fixed';
        var top = parseInt(getComputedStyle(textbox).top);
        setTimeout(function(){
            textbox.style.top = top - (windowHeight - window.innerHeight) + 'px';
            send.style.top = textbox.style.top;
        },100);
    } else {
        document.getElementsByClassName('nav')[3].style.position = 'absolute';
        document.getElementsByClassName('clear-text')[0].style.position = 'absolute';
        textbox.style.position = 'absolute';
        send.style.position = 'absolute';
        document.getElementById('console').style.position = 'absolute';
        textbox.style.top = (windowHeight - 60) + 'px'
        send.style.top = textbox.style.top;
    }
}

var textbox = document.getElementById('txBar');
var textarea = document.getElementById('console');
textarea.value = "";
var command = document.getElementById('command');
var send = document.getElementById('send');
var scroll_h = textarea.scrollHeight;

function keyin()
{
    textbox.addEventListener('keypress', (e) => {
        if(window.event.keyCode == 13) {
            message_send(textbox.value);
            textarea.value += ('S: ' + textbox.value + '\n');
            textbox.value = '';
            textarea.scrollTop = textarea.scrollHeight;
            console.log(textarea.scrollHeight);
        }
    })

    send.addEventListener('click', (e) => {
        if(textbox.value != "") {
            message_send(textbox.value);
            textarea.value += ('S: ' + textbox.value + '\n');
            textbox.value = '';
            textarea.scrollTop = textarea.scrollHeight;
            console.log(textarea.scrollHeight);
        }
    })

    document.getElementsByClassName('clear-text')[0].addEventListener('click', (e) => {
        if(confirm("Clear it all?")) {
            textbox.value = '';
            textarea.value = '';
        }
    })

    textarea.addEventListener('touchstart', (e) => {
        if(textarea.scrollHeight != scroll_h)
            e.stopPropagation();
    })
    textarea.addEventListener('touchmove', (e) => {
        if(textarea.scrollHeight != scroll_h)
            e.stopPropagation();
    })
}

var height = document.getElementById('height');
var speed = document.getElementById('speed');
var apogee = document.getElementById('apogee');
var state = document.getElementById('state');
var history_height = 0;
var mess = '';
var file;

function socket_connect() {
    socket = new WebSocket('ws://' + window.location.hostname + ':81/');

    socket.onopen = function () {
        socket.send("Connected");
    }

    socket.onclose = function (e) {
        console.log('Socket is closed. Reconnecting...', e.reason);
        socket_connect();
        socket_receive();
    }

    socket.onerror = function (err) {
        console.log('Socket encounter error:', err.message, '. Closing socket...');
        socket.close();
    }
}

function socket_receive() {
    socket.onmessage = function (event) {
        var data = event.data.split(',');
        console.log(data);

        // Type:data[0], Time:data[1], Height:data[2], Speed:data[3], Status:data[4], dB:data[5]
        if (data[0] == 's' || data[0] == 'f') {
            height.innerHTML = data[2];
            speed.innerHTML = data[3];
            if (data[4] == '0') state.innerHTML = 'Unknown';
            else if (data[4] == '1') state.innerHTML = 'Rising';
            else if (data[4] == '2') state.innerHTML = 'Falling';
            document.getElementsByClassName('signal-strength')[0].innerHTML = data[5];

            if (parseFloat(data[2]).toFixed(2) > history_height) {
                apogee.innerHTML = data[2];
                history_height = parseFloat(data[2]).toFixed(2);
            }
        }
        if (data[0] != 's') {
            if (data[0] == '\n')
                textarea.value += event.data.substring(2, event.data.length - 1) + "\n";
            else
                textarea.value += 'R: ' + event.data + "\n";
            textarea.scrollTop = textarea.scrollHeight;
        }
        if (data[0] == 'stop') {
            clearInterval(t_plus_ID);
        }
        if(data[0] == 'l') {
            file = data[1].split('\n');
        }
    }
}

function rbf_send() {
    socket.send('RBF');
}

function angle_send(angle) {
    socket.send('angle' + angle);
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

var RBF_btn = document.getElementById('RBF_btn');
var init_btn = document.getElementById('init_btn');
var stall_btn = document.getElementById('stall_btn');
var angle_btn = document.getElementById('angle_btn');
var launch_btn = document.getElementById('launch_btn');
var abort_btn = document.getElementById('abort');
var chute_btn = document.getElementById('parachute');
var verification = { a: false, b: false, c: false, d: false, e: false };
var intervalID;

var test_btn = document.getElementById('test');
var stop_test_btn = document.getElementById('stop-test');

var read_btn = document.getElementById('file-read');
var download_btn = document.getElementById('file-download');

var angle;

function button() {
    RBF_btn.addEventListener('click', (e) => {
        console.log('remove before flight');
        if (confirm("Are you sure you have removed before flight?")) {
            rbf_send();
            console.log('RBF');
            RBF_btn.style.background = '#00cc00';
            verification.a = true;
        } else {
            console.log('no RBF');
        }

    })

    angle_btn.addEventListener('click', (e) => {
        console.log('set count-down time');
        angle = prompt("Please set the countdown time:", 10);
        angle_send(angle);
        verification.b = true;
    })

    init_btn.addEventListener('click', (e) => {
        console.log('initialize');
        if (confirm("Are you sure to initialize?")) {
            init_send();
            console.log('init');
            init_btn.style.background = '#00cc00'
            verification.c = true;
        } else {
            console.log('no init');
        }
    })

    stall_btn.addEventListener('click', (e) => {
        console.log('stallation');
        if (confirm("Are you sure you have stalled the rocket well? \n" +
            "Are you sure that camera men are all in position?")) {
            stall_send();
            console.log('stall');
            stall_btn.style.background = '#00cc00'
            verification.d = true;
        } else {
            console.log('no stall');
        }
    })

    launch_btn.addEventListener('click', (e) => {
        if (verification.a && verification.b && verification.c && verification.d) {
            if (confirm("Are you sure we are go for launch?")) {
                console.log('launch');
                preLaunch_send();
                launch_btn.style.background = '#00cc00';
                verification.e = true;
                count_down();
            } else {
                console.log('no launch');
            }
        } else {
            alert('Something not right...');
        }
    })

    abort_btn.addEventListener('click', (e) => {
        abort_send();
        document.getElementsByClassName('count-down-text')[0].style.fontSize = '80px';
        document.getElementsByClassName('count-down-text')[0].innerHTML = 'Abort'
        clearInterval(intervalID);
        navigator.vibrate(100);
    })

    chute_btn.addEventListener('click', (e) => {
        parachute_send();
        navigator.vibrate(100);
    })

    test_btn.addEventListener('click', (e) => {
        socket.send('test');
    })
    stop_test_btn.addEventListener('click', (e) => {
        socket.send('stop test');
    })

    var read_on = false;
    read_btn.addEventListener('touchstart', (e) => {
        read_on = !read_on;
        if (read_on)
            socket.send('list');

    })
    read_btn.addEventListener('touchend', (e) => {
        if (read_on) {
            setTimeout(function () {
                read_btn.innerHTML = "Read";
                for (var i = 0; i < file.length; i++) {
                    var opt = document.createElement('option');
                    opt.value = file[i];
                    opt.innerHTML = file[i];
                    read_btn.appendChild(opt);
                }
            }, 100);
        }
    })

    var download_on = false;
    download_btn.addEventListener('touchstart', (e) => {
        download_on = !download_on;
        if (download_on)
            socket.send('list');

    })
    download_btn.addEventListener('touchend', (e) => {
        if (download_on) {
            setTimeout(function () {
                download_btn.innerHTML = "Read";
                for (var i = 0; i < file.length; i++) {
                    var opt = document.createElement('option');
                    opt.value = file[i];
                    opt.innerHTML = file[i];
                    download_btn.appendChild(opt);
                }
            }, 100);
        }
    })
}

function goto() {
    var index = read_btn.selectedIndex;
    var option = read_btn.options;
    window.location.href = "/" + option[index].text;
}

function download() {
    var index = download_btn.selectedIndex;
    var option = download_btn.options;
    var element = document.createElement('a');
    element.setAttribute('href', "/" + option[index].text);
    element.setAttribute('download', option[index].text);
    document.body.appendChild(element);
    element.click();
}

function count_down() {
    var countDown = document.getElementsByClassName('count-down-text')[0];
    var countdown_id = document.getElementById('count-down');
    countdown_id.style.left = '0px';
    countdown_id.style.opacity = 1;
    countDown.innerHTML = angle;
    intervalID = setInterval(function () {
        if (countDown.innerHTML == 'Ignition') countDown.innerHTML = 0;
        countDown.innerHTML -= 1;
        if (countDown.innerHTML == 0) {
            countDown.innerHTML = 'Ignition';
            countDown.style.fontSize = '80px';
        } else if (countDown.innerHTML == -1) {
            launch_send();
            clearInterval(intervalID);
            countDown.innerHTML = 'Launch';
            page.style.transition = 'all 1s';
            page.style.left = -screen.width + 'px';
            setTimeout(function () {
                countdown_id.style.display = 'none';
            }, 1000);
            T_plus();
        }
    }, 1000);
}

var t_plus_ID;

function T_plus() {
    var flight_time = document.getElementById('flight_time');
    var time = 0;
    t_plus_ID = setInterval(function () {
        time += 0.01;
        flight_time.innerHTML = time.toFixed(2);
    }, 10);
}

var textbox = document.getElementById('txBar');
var textarea = document.getElementById('console');
textarea.value = "";
var command = document.getElementById('command');
var send = document.getElementById('send');
var scroll_h = textarea.scrollHeight;

function keyin()
{
    textbox.addEventListener('keypress', (e) => {
        if(window.event.keyCode == 13) {
            message_send(textbox.value);
            textarea.value += ('S: ' + textbox.value + '\n');
            textbox.value = '';
            textarea.scrollTop = textarea.scrollHeight;
            console.log(textarea.scrollHeight);
        }
    })

    send.addEventListener('click', (e) => {
        if(textbox.value != "") {
            message_send(textbox.value);
            textarea.value += ('S: ' + textbox.value + '\n');
            textbox.value = '';
            textarea.scrollTop = textarea.scrollHeight;
            console.log(textarea.scrollHeight);
        }
    })

    document.getElementsByClassName('clear-text')[0].addEventListener('click', (e) => {
        if(confirm("Clear it all?")) {
            textbox.value = '';
            textarea.value = '';
        }
    })

    textarea.addEventListener('touchstart', (e) => {
        if(textarea.scrollHeight != scroll_h)
            e.stopPropagation();
    })
    textarea.addEventListener('touchmove', (e) => {
        if(textarea.scrollHeight != scroll_h)
            e.stopPropagation();
    })
}

var height = document.getElementById('height');
var speed = document.getElementById('speed');
var apogee = document.getElementById('apogee');
var state = document.getElementById('state');
var history_height = 0;
var mess = '';
var file;

function socket_connect() {
    socket = new WebSocket('ws://' + window.location.hostname + ':81/');

    socket.onopen = function () {
        socket.send("Connected");
    }

    socket.onclose = function (e) {
        console.log('Socket is closed. Reconnecting...', e.reason);
        socket_connect();
        socket_receive();
    }

    socket.onerror = function (err) {
        console.log('Socket encounter error:', err.message, '. Closing socket...');
        socket.close();
    }
}

function socket_receive() {
    socket.onmessage = function (event) {
        var data = event.data.split(',');
        console.log(data);

        // Type:data[0], Time:data[1], Height:data[2], Speed:data[3], Status:data[4], dB:data[5]
        if (data[0] == 's' || data[0] == 'f') {
            height.innerHTML = data[2];
            speed.innerHTML = data[3];
            if (data[4] == '0') state.innerHTML = 'Unknown';
            else if (data[4] == '1') state.innerHTML = 'Rising';
            else if (data[4] == '2') state.innerHTML = 'Falling';
            document.getElementsByClassName('signal-strength')[0].innerHTML = data[5];

            if (parseFloat(data[2]).toFixed(2) > history_height) {
                apogee.innerHTML = data[2];
                history_height = parseFloat(data[2]).toFixed(2);
            }
        }
        if (data[0] != 's') {
            if (data[0] == '\n')
                textarea.value += event.data.substring(2, event.data.length - 1) + "\n";
            else
                textarea.value += 'R: ' + event.data + "\n";
            textarea.scrollTop = textarea.scrollHeight;
        }
        if (data[0] == 'stop') {
            clearInterval(t_plus_ID);
        }
        if(data[0] == 'l') {
            file = data[1].split('\n');
        }
    }
}

function rbf_send() {
    socket.send('RBF');
}

function angle_send(angle) {
    socket.send('angle' + angle);
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