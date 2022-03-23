var RBF_btn = document.getElementById('RBF_btn');
var init_btn = document.getElementById('init_btn');
var stall_btn = document.getElementById('stall_btn');
var angle_btn = document.getElementById('angle_btn');
var launch_btn = document.getElementById('launch_btn');
var abort_btn = document.getElementById('abort');
var chute_btn = document.getElementById('parachute');
var verification = {a: false, b: false, c: false, d: false, e:false};
var intervalID;

var test_btn = document.getElementById('test');
var stop_test_btn = document.getElementById('stop-test');

function button()
{
    RBF_btn.addEventListener('click', (e) => {
        console.log('remove before flight');
        if(confirm("Are you sure you have removed before flight?")){
            rbf_send();
            console.log('RBF');
            RBF_btn.style.background = '#00cc00';
            verification.a = true;
        } else {
            console.log('no RBF');
        }
        
    })

    angle_btn.addEventListener('click', (e) => {
        console.log('angle set');
        var angle = prompt("Please enter stalling angle:", 4);
        console.log('set angle to ' + angle + ' degree');
        angle_send(angle);
        verification.b = true;
    })

    init_btn.addEventListener('click', (e) => {
        console.log('initialize');
        if(confirm("Are you sure to initialize?")){
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
        if(confirm("Are you sure you have stalled the rocket well? \n"+
                   "Are you sure that camera men are all in position?")){
            stall_send();
            console.log('stall');
            stall_btn.style.background = '#00cc00'
            verification.d = true;
        } else {
            console.log('no stall');
        }
    })

    launch_btn.addEventListener('click', (e) => {
        if(verification.a && verification.b && verification.c && verification.d) {
            if(confirm("Are you sure we are go for launch?")){
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
}

function count_down()
{
    var countDown = document.getElementsByClassName('count-down-text')[0];
    var countdown_id = document.getElementById('count-down');
    countdown_id.style.left = '0px';
    countdown_id.style.opacity = 1;
    intervalID = setInterval(function(){
        if(countDown.innerHTML == 'Ignition') countDown.innerHTML = 0;
        countDown.innerHTML -= 1;
        if(countDown.innerHTML == 0) {
            countDown.innerHTML = 'Ignition';
            countDown.style.fontSize = '80px';
        } else if(countDown.innerHTML == -1) {
            launch_send();
            clearInterval(intervalID);
            countDown.innerHTML = 'Launch';
            page.style.transition = 'all 1s';
            page.style.left = -screen.width + 'px';
            setTimeout(function(){
                countdown_id.style.display = 'none';
            }, 1000);
            T_plus();
        }
    }, 1000);
}

var t_plus_ID;

function T_plus()
{
    var flight_time = document.getElementById('flight_time');
    var time = 0;
    t_plus_ID = setInterval(function(){
        time += 0.01;
        flight_time.innerHTML = time.toFixed(2);
    }, 10);
}