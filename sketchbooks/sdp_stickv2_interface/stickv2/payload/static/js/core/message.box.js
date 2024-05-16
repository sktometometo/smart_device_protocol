var message_box_flag = 0;
function messageBox(title,message){

    if(message_box_flag == 1){
        message_box_remove();
    }

    var body = document.querySelector("body");

    var box =  document.createElement("div");
    box.setAttribute("id",'message_box');
    box.setAttribute("class","message-box");

    var box_title = document.createElement("div");
    box_title.setAttribute("class","message-box-title");
    box_title.innerHTML = `
        <div class="message-box-title-text">`+ title +`</div>
        <div class="message-box-title-close">
        <svg t="1617790180115" class="icon" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg" p-id="2052"
        width="32" height="32">
        <path
            d="M697.173333 85.333333H327.253333C182.613333 85.333333 85.333333 186.88 85.333333 337.92v348.586667C85.333333 837.12 182.613333 938.666667 327.253333 938.666667h369.92c144.64 0 241.493333-101.546667 241.493334-252.16V337.92C938.666667 186.88 841.813333 85.333333 697.173333 85.333333z"
            fill="#ffffff" opacity=".4" p-id="2053"></path>
        <path
            d="M640.682667 587.52l-75.946667-75.861333 75.904-75.861334a37.290667 37.290667 0 0 0 0-52.778666 37.205333 37.205333 0 0 0-52.778667 0l-75.946666 75.818666-75.861334-75.946666a37.418667 37.418667 0 0 0-52.821333 0 37.418667 37.418667 0 0 0 0 52.821333l75.946667 75.946667-75.776 75.733333a37.290667 37.290667 0 1 0 52.778666 52.821333l75.776-75.776 75.946667 75.946667a37.376 37.376 0 0 0 52.778667-52.821333z"
            fill="#ffffff" p-id="2054"></path>
        </svg>
        </div>
    `
    var box_message = document.createElement("div");
    box_message.setAttribute("class","message-box-body");
    box_message.innerHTML = message;
    var box_menu = document.createElement("div");
    box_menu.setAttribute("class","message-box-menu");
    box_menu.innerHTML = `
        <button type="button" onclick="message_box_yes()">yes</button>
        <button type="button" onclick="message_box_no()">no</button>
    `
    box.appendChild(box_title);
    box.appendChild(box_message);
    box.appendChild(box_menu);
    body.appendChild(box);
    message_box_flag = 1;
}

function message_box_yes(){
    message_box_remove()
    return true;
}

function message_box_no(){
    message_box_remove()
    return false;
}

function message_box_remove(){
    if(document.getElementById("message_box") != null){
        document.getElementById("message_box").parentNode.removeChild(document.getElementById("message_box"));
    }   
    message_box_flag = 0;
}