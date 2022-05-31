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