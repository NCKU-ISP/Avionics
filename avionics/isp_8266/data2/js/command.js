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