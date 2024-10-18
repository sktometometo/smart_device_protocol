
//设置开机启动功能
function SetUpBootFunc(boot_func_name){
    $.ajax({
        url:'/system_config',
        type:'post',
        contentType: "application/json;charset=UTF-8",
        data:JSON.stringify({"boot_func":boot_func_name}),
        success:function(res){
            alert('updated success.')
        },
        error:function(res){
            alert('updated failed.')
        }
    })
}
//获取开机启动功能
function getBootFunc(){  
    $.ajax({
        url:'/get_system_config',
        type:'post',
        success:function(res){
            try {
                let data = JSON.parse(res);
                if(res != null){     
                    boot_name = data.boot_func;
                    $(".canvas-func-title").html(res.title);
                    $(".canvas-func-p").html(res.description);  
                    if(func_init == 1) {  
                        $("#"+boot_name).click();   
                     }  
                    bootFuncSelect(boot_name);
                }     
              
                yolo_last_model_path = data.yolo_last_model_path
            } catch (error) {
                console.error(error);
            }
   
        },
        error:function(res){
            
        },
        complete:function(){
            func_init = 0;
        }
    })

  

}

function getLastFunc(){
    initFuncMenu();
    func_init = 1;
    $.ajax({
        url:'/get_last_func',
        type:'post',
        dataType:"json",
        async:false,
        success:function(res){
            if(res != null){
                func_name =res.last_func;
                $(".canvas-func-title").html(res.title);
                $(".canvas-func-p").html(res.description);
                if(func_name != null){
                    $("#"+func_name).click();  
                    func_init = 0;
                }            
            }else{         
            }      
        },
        error:function(res){
        },
    })
}

function book_func_init(){
    $(".func-body .boot-func button svg").css({
        "transform": "rotate(90deg)",
    })
    $("button#func-header-btn img").on('click',function(){
        $(".func-panel").css({
            "right" : "-450px",
            "transition" : "right .25s linear"
        }); 
    })

    $("#func_model img").on('click',function(){
        $(".func-panel").css({
            "right" : "0",
            "transition" : "right .25s linear"
        })
    })
    $(".func-body .boot-func button").on('click',function(){
        if(toggle_defalut_func_list == 0){
            $(".boot-func-list").fadeIn("fast");
            $(".func-body .boot-func button svg").css({
                "transform": "rotate(0deg)",
                "transition" : "transform .25s linear"
            })
            toggle_defalut_func_list = 1;
        }else{
            $(".boot-func-list").fadeOut("fast");
            $(".func-body .boot-func button svg").css({
                "transform": "rotate(90deg)",
            })
            toggle_defalut_func_list = 0;
        }

    })
    $(".boot-func-list").find('button').on('click',function(){       
        let clazz = $(this).attr('class');
        let value = $(this).html();
        let func = $(this).attr("id").split('-');
        type_name = func[0];
        $(".func-body .boot-func button").attr('class',clazz);
        $(".func-body .boot-func button span").html(value)
        //设置默认开机功能
        SetUpBootFunc(type_name)
    })

}
function bootFuncSelect(boot_func){
    var index = $.inArray(boot_func,func_list)
    if(index >=0){
        $(".boot-func-list").find('button').each(function(){
            let func = $(this).attr("id").split('-');
            type_name = func[0];
            if(type_name == boot_func){
                let clazz = $(this).attr('class');
                let value = $(this).html();
                $(".func-body .boot-func button").attr('class',clazz);
                $(".func-body .boot-func button span").html(value)                
             }      
        })
    }
}