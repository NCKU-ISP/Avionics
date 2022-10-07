function socket_connect() {
    // socket = new WebSocket('ws://' + window.location.hostname + ':81/');
    socket = new WebSocket('ws://' + '192.168.4.1' + ':81/');

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
            file[0] = file[0].substring(1);
            file[file.length-1] = file[file.length-1].substring(0,file[file.length-1].length-1);
        }
    }
}