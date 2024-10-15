$(function(){
    //初始化动画效果和监听事件
    init();
    //获取最后运行的功能
    getLastFunc()
    //获取开机默认功能
    getBootFunc();
    // //加载数据流
    // loadStream()
    //请求画面渲染数据
    requestStreamData();
    //请求结果数据
    requestFuncResult();
    //请求音频数据
    requset_meta();
})