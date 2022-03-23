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
        var threshould = screen.width / 4;

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
                if(change.y > 400) window.location.reload();
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