/**
 *canvas 左边的结果显示.
 * */
 var func_result;
 function funcResultInit(){
    requestFuncResult()
}

 //请求 渲染视频流的数据
 function requestFuncResult() {
     fetch('/func/result',{method:"POST"}).then((response)=>{
         fetchFuncResult(response.body)
     }).catch(error=>{
         console.log("requestFuncResult error " +error)
     })
 }
 function fetchFuncResult(stream) {
     const reader = stream.getReader(); 
     // read() 返回了一个promise
     // 当数据被接收时resolve
     reader.read().then(function processText({ done, value }) {
       // Result对象包含了两个属性:
       // done  - 当stream传完所有数据时则变成true
       // value - 数据片段。当done不为true时永远为undefined
       let str = String.fromCharCode.apply(null, value);
       if(str!=null && str.length>0){
         let res =str;
         if(res != null){
             res = str.split('|')
             funcResultShow(res[0]);
         } else{
 
         }
       }else{
         
       }
       if (done) {
         console.log("Func complete");
         return;
       }
       // 再次调用这个函数以读取更多数据
       return reader.read().then(processText);
     });
   }

    //code detector
    //items: []

   function funcResultShow(res){
      try{
        $(".func-result #func-result-pre").html(JSON.stringify(JSON.parse(res),null,4));
      }catch(e){

      }
   }
 