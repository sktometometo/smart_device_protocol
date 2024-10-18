function wifi_connect(){
    ssid = $("#wifi-ssid").val();
    password = $("#wifi-password").val();
    if(ssid == null || ssid == '' || password == null || password == ''){
        alert("ssid or password cannot be empty!")
        return;
    }
    $.ajax({
        type:'post',
        url:'/wifi_connect',
        contentType:'application/json;charset=UTF-8',
        data:{
            "ssid":ssid,
            "password":password
        },
        success:function(res){
            

        },
        error:function(res){
            alert("wifi connection failed,please login again!")
        }
    })
}