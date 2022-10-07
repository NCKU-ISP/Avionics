var RBF_btn = document.getElementById('RBF_btn');
var init_btn = document.getElementById('init_btn');
var stall_btn = document.getElementById('stall_btn');
var angle_btn = document.getElementById('angle_btn');
var launch_btn = document.getElementById('launch_btn');
var abort_btn = document.getElementById('abort');
var chute_btn = document.getElementById('parachute');
var emergency_btn = document.getElementById('emergency');
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
        navigator.vibrate(100);
        abort_send();
        document.getElementsByClassName('count-down-text')[0].style.fontSize = '80px';
        document.getElementsByClassName('count-down-text')[0].innerHTML = 'Abort'
        clearInterval(intervalID);
    })

    chute_btn.addEventListener('click', (e) => {
        navigator.vibrate(100);
        parachute_send();
    })

    var emergency_clicked = 0;
    emergency_btn.addEventListener('click', (e) => {
        navigator.vibrate(200);
        emergency_clicked++;
        if(emergency_clicked == 3){
            emergency_clicked = 0;
            emergency_send();
        }
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