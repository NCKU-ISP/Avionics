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

        if (dir == 1) {
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
        } else if (dir == 2) {
            if (now.y == 0 && change.y > 0 || now.y == -1 * window.innerHeight && change.y < 0) {
                if (change.y > 400) window.location.replace('/index.html');
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

function command_fixed(input) {
    if (input) {
        document.getElementsByClassName('nav')[3].style.position = 'fixed';
        textbox.style.position = 'fixed';
        send.style.position = 'fixed';
        document.getElementById('console').style.position = 'fixed';
        document.getElementsByClassName('clear-text')[0].style.position = 'fixed';
        var top = parseInt(getComputedStyle(textbox).top);
        setTimeout(function () {
            textbox.style.top = top - (windowHeight - window.innerHeight) + 'px';
            send.style.top = textbox.style.top;
        }, 100);
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

function keyin() {
    textbox.addEventListener('keypress', (e) => {
        if (window.event.keyCode == 13) {
            message_send(textbox.value);
            textarea.value += ('S: ' + textbox.value + '\n');
            textbox.value = '';
            textarea.scrollTop = textarea.scrollHeight;
            console.log(textarea.scrollHeight);
        }
    })

    send.addEventListener('click', (e) => {
        if (textbox.value != "") {
            message_send(textbox.value);
            textarea.value += ('S: ' + textbox.value + '\n');
            textbox.value = '';
            textarea.scrollTop = textarea.scrollHeight;
            console.log(textarea.scrollHeight);
        }
    })

    document.getElementsByClassName('clear-text')[0].addEventListener('click', (e) => {
        if (confirm("Clear it all?")) {
            textbox.value = '';
            textarea.value = '';
        }
    })

    textarea.addEventListener('touchstart', (e) => {
        if (textarea.scrollHeight != scroll_h)
            e.stopPropagation();
    })
    textarea.addEventListener('touchmove', (e) => {
        if (textarea.scrollHeight != scroll_h)
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



function goto() {
    var index = read_btn.selectedIndex;
    var option = read_btn.options;
    window.location.assign('http://192.168.4.1/' + option[index].text);
}

function download() {
    var index = download_btn.selectedIndex;
    var option = download_btn.options;

    fetch('http://192.168.4.1/' + option[index].text)
        .then(response => response.blob())
        .then(blob => {
            const link = document.createElement("a");
            link.href = URL.createObjectURL(blob);
            link.download = option[index].text;
            link.click();
        })
        .catch(console.error);
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

function keyin() {
    textbox.addEventListener('keyup', (e) => {
        console.log(window.event.keyCode);
        if (window.event.keyCode == 13) {
            console.log(textarea.scrollHeight);
            message_send(textbox.value);
            textarea.value += ('S: ' + textbox.value + '\n');
            textbox.value = '';
            textarea.scrollTop = textarea.scrollHeight;
        }
    })

    send.addEventListener('click', (e) => {
        if (textbox.value != "") {
            message_send(textbox.value);
            textarea.value += ('S: ' + textbox.value + '\n');
            textbox.value = '';
            textarea.scrollTop = textarea.scrollHeight;
            console.log(textarea.scrollHeight);
        }
    })

    document.getElementsByClassName('clear-text')[0].addEventListener('click', (e) => {
        if (confirm("Clear it all?")) {
            textbox.value = '';
            textarea.value = '';
        }
    })

    textarea.addEventListener('touchstart', (e) => {
        if (textarea.scrollHeight != scroll_h)
            e.stopPropagation();
    })
    textarea.addEventListener('touchmove', (e) => {
        if (textarea.scrollHeight != scroll_h)
            e.stopPropagation();
    })
}