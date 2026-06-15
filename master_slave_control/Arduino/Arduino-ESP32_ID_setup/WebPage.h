const char index_html[] PROGMEM = R"rawliteral(

<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>RoArm-M1</title>
<style type="text/css">
    
    body {
        background-image: linear-gradient(#3F424F, #1E212E);
        background-image: -o-linear-gradient(#3F424F, #1E212E);
        background-image: -moz-linear-gradient(#3F424F, #1E212E);
        background-image: -webkit-linear-gradient(#3F424F, #1E212E);
        background-image: -ms-linear-gradient(#3F424F, #1E212E);
        font-family: "roboto",helt "sans-serif";
        font-weight: lighter;
        color: rgba(216,216,216,0.8)
        background-size: cover;
        background-position: center 0;
        background-attachment: fixed;
        color: rgba(255,255,255,0.6);
        border: 0px;
        margin: 0;
        padding: 0;
    }
    main{
        width: 1140px;
        margin: auto;
        margin-bottom: 80px;
    }
    main > div{
        width: 100%;
        display: flex;
        justify-content: space-between;
        padding: 54px 0;
    }
    main > div:first-child{border-bottom: 1px dashed rgba(216,216,216,0.24)}
    section > div{
        width: 516px;
    }

    button {
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;

        user-select: none;
    }
    .btn-all, .btn-of, .btn-num, .btn-num-lr, .servo-btn-num, .btn-stop, .btn-set{cursor: pointer}
    .tittle-h2{
        font-weight:lighter;
        font-size: 24px;
        color: rgba(255,255,255,0.8)
    }
    .sec-1, .sec-3{
        width: 50%;
        border-right: 1px dashed rgba(216,216,216,0.24);
    }
    .set-btn-frame{
        display: flex;
        justify-content: space-between;
        align-items: flex-end;
    }
    .set-btn-frame-i{
        width: 360px;
        display: inline-flex;
        justify-content: space-between;
        text-align: center;
    }
    .set-btn-sections{width: 48px;}
    .btn-num{
        width: 48px;
        height: 48px;
        margin: 1px 0 0 0;
        font-size: 15px;
        font-weight: lighter;
    }
    .btn-num-lr{
        width: 48px;
        height: 46px;
        font-size: 15px;
    }
    .btn-num-up{border-radius: 4px 4px 0 0}
    .btn-num-down{border-radius: 0 0 4px 4px}
    .btn-num-bg{
        background-color: rgba(255,255,255,0.06);
        border: none;
        color: rgba(255,255,255,0.5);
    }
    .btn-num-bg:hover{background-color: rgba(255,255,255,0.02);}
    .btn-all, .btn-of{
        background-color: rgba(164,169,186,0.25);
        border: none;
        border-radius: 4px;
        font-size: 15px;
        color: rgba(255,255,255,0.5);
        font-weight: lighter;
    }
    .btn-of-active{
        background-color: rgba(38,152,234,0.1);
        color: #1EA1FF;
        border: 1px solid #1EA1FF;
        border-radius: 4px;
        font-size: 15px;
        
    }
    .btn-all:hover, .btn-of:hover{background-color:rgba(164,169,186,0.15);}
    .btn-main-l{
        width: 126px;
        height: 97px;
    }
    .init-posit{
        vertical-align: bottom;
    }
    .record-mum-set > div {
        display: flex;
        justify-content: space-between;
        margin-top: 54px;
    }
    .record-mum-set > div > div:last-child {
        width: 320px;
        display: flex;
        justify-content: space-between;
        align-items: flex-start;
    }
    .record-mum-set > div > div:last-child > div {
        display: flex;
        width: 164px;
        border: 1px solid rgba(206,206,206,0.15);
        border-radius: 4px;
    }
    .num-insert{
        background-color: rgba(0,0,0,0.01);
        border: 0px;
        width: 68px;
        text-align: center;
        color: rgba(255,255,255,0.8);
        font-size: 15px;
        font-weight: lighter;
    }
    .btn-main-s{
        width: 126px;
        height: 48px;
    }
    .two-btn > div:first-child{
        display: flex;
        justify-content: space-between;
    }
    .btn-main-m{
        width: 204px;
        height: 48px;
    }
    .two-btn1{
        border-top:1px dashed rgba(216,216,216,0.24) ;
        border-bottom:1px dashed rgba(216,216,216,0.24) ;
        padding: 30px 0;
        margin: 30px 0;
    }
    .Servo-set > div{
        display: flex;
        justify-content: space-between;
    }
    .servo-btn-num{
        height: 48px;
        width: 126px;
        border-radius: 4px;
        font-size: 15px;
        font-weight: lighter;
    }
    .servo-btn-num + button{margin-left: 30px;}
    .Servo-set{margin: 30px 0;}
    .sec-5{
        position: fixed;
        bottom: 0px;
        width: 100%;
        display: flex;
        justify-content: center;
        padding: 40px 0;
        background-image: linear-gradient(rgba(30,33,46,0),rgba(30,33,46,1));
        
    }
    .sec-5 button{margin: 0 15px;}
    .btn-stop{
        width: 204px;
        height: 48px;
        background-color: rgba(181,104,108,1);
        color: white;
        border-radius: 1000px;
        border: none;
    }
    .btn-set{
        width: 48px;
        height: 48px;
        background-color: rgba(115,134,151,1);
        border-radius: 1000px;
        border: none;
    }
    .btn-set:hover{background-color: rgba(115,134,151,0.5);}
    .record-tt{
        color: rgba(255,255,255,0.5);
        font-size: 15px;
        
    }
    .record-height{height: 1px;}
    .sec-3text{
        font-size: 15px;
        text-align: center;
        color: rgba(255,255,255,0.4)
    }
    .btn-stop:hover{background-color: rgba(181,104,108,0.5);}
    input::-webkit-outer-spin-button,input::-webkit-inner-spin-button {
        -webkit-appearance: none;
    }
    input[type='number']{
        -moz-appearance: textfield;
    }
    .sec-3text p{word-break:break-all;}
    @media screen and (min-width: 768px) and (max-width: 1200px){
        main{
            width: 516px;
            display: block;
            margin-bottom: 150px;
        }
        .sec-1, .sec-3{width: 100%}
        main > div{display: block;}
        .sec-1, .sec-3{border: none;}
        main > div{padding: 0;}
        main section{
            margin-bottom: 30px;
            border-bottom: 1px dashed rgba(216,216,216,0.24);
            border-top: 1px dashed rgba(216,216,216,0.24);
        }
        .sec-4{border-bottom: none;}
        .record-mum-set > div{margin-top: 30px;}
        main > div:first-child{border: none;}
        .sec-2{padding-bottom: 30px;}
    }
    @media screen and (min-width: 360px) and (max-width: 767px){
        main{
            width: 94%;
            display: block;
        }
        .sec-1, .sec-3{width: 100%}
        main > div{display: block;}
        .sec-1, .sec-3{border: none;}
        section > div{width: auto;}
        .set-btn-frame{
            display: block;
        }
        .set-btn-frame-i{width: 100%;}
        .btn-main-l{
            width: 100%;
            height: 48px;
        }
        .init-posit{
            margin-top: 30px;
        }
        .record-mum-set > div{display: block;}
        .record-mum-set > div > div:last-child{
            width: 100%;
        }
        .sec-5 button{
            margin: 0 4px;
        }
        .two-btn button:first-child{margin-right: 10px;}
        .Servo-set > div > div{
            display: flex;
            width: 66.666666%;
            margin-right: 10px
        }
        .servo-btn-num{
            flex: 1;
        }
        .btn-main-s{width: 33.333%;}
        .servo-btn-num + button{margin-left: 10px;}
        .btn-main-m{
            flex: 1;
            width: auto;
        }
        main > div{padding: 10px 0}
        .record-mum-set > div{margin:  30px 0}
        main section{
            margin-bottom: 30px;
            border-bottom: 1px dashed rgba(216,216,216,0.24);
            border-top: 1px dashed rgba(216,216,216,0.24);
        }
        .sec-4{border-bottom: none;}
        .record-mum-set > div{margin-top: 30px;}
        main > div:first-child{border: none;}
        .record-height{display: none;}
    }
</style>
    <script type="text/javascript">
        function opera(x, y) {
            var rs = new Number(document.getElementById(x).value);

            if (y) {
                document.getElementById(x).value = rs + 1;
            } else if( rs >0) {
                document.getElementById(x).value = rs - 1;
            }
	   }
        function opera1(x, y) {
            var rs = new Number(document.getElementById(x).value);

            if (y) {
                document.getElementById(x).value = rs + 1;
            } else if( rs >-1) {
                document.getElementById(x).value = rs - 1;
            }
	   }
        function opera100(x, y) {
            var rs = new Number(document.getElementById(x).value);

            if (y) {
                document.getElementById(x).value = rs + 100;
            } else if( rs >0) {
                document.getElementById(x).value = rs - 100;
            }
	   }
    </script>
</head>

<body>
    <main>
        <div>
            <section class="sec-1">
                <div>
                    <div>
                        <h2 class="tittle-h2">AngleCtrl</h2>
                        <div class="set-btn-frame">
                            <div class="set-btn-frame-i">
                                <div class="set-btn-sections">
                                    <p id="anglectrl-t1">100</p>
                                    <div>
                                        <button class="btn-num btn-num-bg btn-num-up" onmousedown="cmdSend(1,1,1);" ontouchstart="cmdSend(1,1,1);" onmouseup="cmdSend(1,1,3);" ontouchend="cmdSend(1,1,3)">1 +</button>
                                        <button class="btn-num btn-num-bg btn-num-down" onmousedown="cmdSend(1,1,2);" ontouchstart="cmdSend(1,1,2);" onmouseup="cmdSend(1,1,3);" ontouchend="cmdSend(1,1,3)">1 -</button>
                                    </div>
                                </div>
                                <div class="set-btn-sections">
                                    <p id="anglectrl-t2">100</p>
                                    <div>
                                        <button class="btn-num btn-num-bg btn-num-up" onmousedown="cmdSend(1,2,1);" ontouchstart="cmdSend(1,2,1);" onmouseup="cmdSend(1,2,3);" ontouchend="cmdSend(1,2,3)">2 +</button>
                                        <button class="btn-num btn-num-bg btn-num-down" onmousedown="cmdSend(1,2,2);" ontouchstart="cmdSend(1,2,2);" onmouseup="cmdSend(1,2,3);" ontouchend="cmdSend(1,2,3)">2 -</button>
                                    </div>
                                </div>
                                <div class="set-btn-sections">
                                    <p id="anglectrl-t3">100</p>
                                    <div>
                                        <button class="btn-num btn-num-bg btn-num-up" onmousedown="cmdSend(1,3,1);" ontouchstart="cmdSend(1,3,1);" onmouseup="cmdSend(1,3,3);" ontouchend="cmdSend(1,3,3)">3 +</button>
                                        <button class="btn-num btn-num-bg btn-num-down" onmousedown="cmdSend(1,3,2);" ontouchstart="cmdSend(1,3,2);" onmouseup="cmdSend(1,3,3);" ontouchend="cmdSend(1,3,3)">3 -</button>
                                    </div>
                                </div>
                                <div class="set-btn-sections">
                                    <p id="anglectrl-t4">100</p>
                                    <div>
                                        <button class="btn-num btn-num-bg btn-num-up" onmousedown="cmdSend(1,4,1);" ontouchstart="cmdSend(1,4,1);" onmouseup="cmdSend(1,4,3);" ontouchend="cmdSend(1,4,3)">4 +</button>
                                        <button class="btn-num btn-num-bg btn-num-down" onmousedown="cmdSend(1,4,2);" ontouchstart="cmdSend(1,4,2);" onmouseup="cmdSend(1,4,3);" ontouchend="cmdSend(1,4,3)">4 -</button>
                                    </div>
                                </div>
                                <div class="set-btn-sections">
                                    <p id="anglectrl-t5">100</p>
                                    <div>
                                        <button class="btn-num btn-num-bg btn-num-up" onmousedown="cmdSend(1,5,1);" ontouchstart="cmdSend(1,5,1);" onmouseup="cmdSend(1,5,3);" ontouchend="cmdSend(1,5,3)">5 +</button>
                                        <button class="btn-num btn-num-bg btn-num-down" onmousedown="cmdSend(1,5,2);" ontouchstart="cmdSend(1,5,2);" onmouseup="cmdSend(1,5,3);" ontouchend="cmdSend(1,5,3)">5 -</button>
                                    </div>
                                </div>
                            </div>
                            <div class="init-posit">
                                <button class="btn-all btn-main-l btn-all-bg" onclick="cmdSend(1,0,0);">INIT</button>
                            </div>
                        </div>
                    </div>
                    <div class="two-btn two-btn1">
                        <div>
                            <button class="btn-of btn-main-m btn-all-bg" onclick="cmdSend(2,0,0);">Torque_off</button>
                            <button class="btn-of btn-main-m btn-all-bg" onclick="cmdSend(2,1,0);">Torque_on</button>
                        </div>
                    </div>
                    <div>
                        <h2 class="tittle-h2">CoordCtrl</h2>
                        <div class="set-btn-frame">
                            <div class="set-btn-frame-i">
                                <div class="set-btn-sections">
                                    <p id="coordCtrl-t1">100</p>
                                    <div>
                                        <button class="btn-num btn-num-bg btn-num-up" onmousedown="cmdSend(3,1,1);" ontouchstart="cmdSend(3,1,1);" onmouseup="cmdSend(3,1,3);" ontouchend="cmdSend(3,1,3)">X +</button>
                                        <button class="btn-num btn-num-bg btn-num-down" onmousedown="cmdSend(3,1,2);" ontouchstart="cmdSend(3,1,2);" onmouseup="cmdSend(3,1,3);" ontouchend="cmdSend(3,1,3)">X -</button>
                                    </div>
                                </div>
                                <div class="set-btn-sections">
                                    <p id="coordCtrl-t2">100</p>
                                    <div>
                                        <button class="btn-num btn-num-bg btn-num-up" onmousedown="cmdSend(3,2,1);" ontouchstart="cmdSend(3,2,1);" onmouseup="cmdSend(3,2,3);" ontouchend="cmdSend(3,2,3)">Y +</button>
                                        <button class="btn-num btn-num-bg btn-num-down" onmousedown="cmdSend(3,2,2);" ontouchstart="cmdSend(3,2,2);" onmouseup="cmdSend(3,2,3);" ontouchend="cmdSend(3,2,3)">Y -</button>
                                    </div>
                                </div>
                                <div class="set-btn-sections">
                                    <p id="coordCtrl-t3">100</p>
                                    <div>
                                        <button class="btn-num btn-num-bg btn-num-up" onmousedown="cmdSend(3,3,1);" ontouchstart="cmdSend(3,3,1);" onmouseup="cmdSend(3,3,3);" ontouchend="cmdSend(3,3,3)">Z +</button>
                                        <button class="btn-num btn-num-bg btn-num-down" onmousedown="cmdSend(3,3,2);" ontouchstart="cmdSend(3,3,2);" onmouseup="cmdSend(3,3,3);" ontouchend="cmdSend(3,3,3)">Z -</button>
                                    </div>
                                </div>
                                <div class="set-btn-sections">
                                    <p id="coordCtrl-t4">100</p>
                                    <div>
                                        <button class="btn-num btn-num-bg btn-num-up" onmousedown="cmdSend(3,4,1);" ontouchstart="cmdSend(3,4,1);" onmouseup="cmdSend(3,4,3);" ontouchend="cmdSend(3,4,3)">T +</button>
                                        <button class="btn-num btn-num-bg btn-num-down" onmousedown="cmdSend(3,4,2);" ontouchstart="cmdSend(3,4,2);" onmouseup="cmdSend(3,4,3);" ontouchend="cmdSend(3,4,3)">T -</button>
                                    </div>
                                </div>
                                <div class="set-btn-sections">
                                    <p id="coordCtrl-t5">100</p>
                                    <div>
                                        <button class="btn-num btn-num-bg btn-num-up" onmousedown="cmdSend(1,5,1);" ontouchstart="cmdSend(1,5,1);" onmouseup="cmdSend(1,5,3);" ontouchend="cmdSend(1,5,3)">G +</button>
                                        <button class="btn-num btn-num-bg btn-num-down" onmousedown="cmdSend(1,5,2);" ontouchstart="cmdSend(1,5,2);" onmouseup="cmdSend(1,5,3);" ontouchend="cmdSend(1,5,3)">G -</button>
                                    </div>
                                </div>
                            </div>
                            <div class="init-posit">
                                <button class="btn-all btn-main-l btn-all-bg" onclick="cmdSend(3,0,0);">INIT</button>
                            </div>
                        </div>
                    </div>
                </div>
            </section>
            <section class="sec-2">
                <div>
                    <div>
                        <h2 class="tittle-h2">Record &amp; Replay</h2>
                        <div class="record-height"></div>
                        <div class="record-mum-set">
                            <div>
                                <p class="record-tt">Record as step</p>
                                <div>
                                    <div>
                                        <button type="button" onclick="opera('recor-mum1', false);" class="btn-num-lr btn-num-bg btn-num-left">-</button>
                                        <input id="recor-mum1" min="0" value="3" type="number" class="num-insert">
                                        <button type="button" onclick="opera('recor-mum1', true);" class="btn-num-lr btn-num-bg btn-num-right">+</button>
                                    </div>
                                    <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(5,1,document.getElementById('recor-mum1').value);">Confirm</button>
                                </div>
                            </div>
                            <div>
                                <p class="record-tt">Move to step</p>
                                <div>
                                    <div>
                                        <button type="button" onclick="opera('recor-mum2', false);" class="btn-num-lr btn-num-bg btn-num-left">-</button>
                                        <input id="recor-mum2" min="0" value="3" type="number" class="num-insert">
                                        <button type="button" onclick="opera('recor-mum2', true);" class="btn-num-lr btn-num-bg btn-num-right">+</button>
                                    </div>
                                    <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(5,2,document.getElementById('recor-mum2').value);">Confirm</button>
                                </div>
                            </div>
                            <div>
                                <p class="record-tt">Remove step</p>
                                <div>
                                    <div>
                                        <button type="button" onclick="opera('recor-mum3', false);" class="btn-num-lr btn-num-bg btn-num-left">-</button>
                                        <input id="recor-mum3" min="0" value="3" type="number" class="num-insert">
                                        <button type="button" onclick="opera('recor-mum3', true);" class="btn-num-lr btn-num-bg btn-num-right">+</button>
                                    </div>
                                    <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(5,3,document.getElementById('recor-mum3').value);">Confirm</button>
                                </div>
                            </div>
                            <div>
                                <p class="record-tt">Record delayTime</p>
                                <div>
                                    <div>
                                        <button type="button" onclick="opera100('recor-mum4', false);" class="btn-num-lr btn-num-bg btn-num-left">-</button>
                                        <input id="recor-mum4" min="0" value="500" type="number" class="num-insert">
                                        <button type="button" onclick="opera100('recor-mum4', true);" class="btn-num-lr btn-num-bg btn-num-right">+</button>
                                    </div>
                                    <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(5,4,document.getElementById('recor-mum4').value);">Confirm</button>
                                </div>
                            </div>
                            <div>
                                <p class="record-tt">Replay loopTime</p>
                                <div>
                                    <div>
                                        <button type="button" onclick="opera1('recor-mum5', false);" class="btn-num-lr btn-num-bg btn-num-left">-</button>
                                        <input id="recor-mum5" value="3" type="number" class="num-insert">
                                        <button type="button" onclick="opera1('recor-mum5', true);" class="btn-num-lr btn-num-bg btn-num-right">+</button>
                                    </div>
                                    <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(5,5,document.getElementById('recor-mum5').value);">Start</button>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </section>
        </div>
        <div>
            <section class="sec-3">
                <div>
                    <div  class="two-btn two-btn2">
                        <div>
                            <button class="btn-all btn-main-m btn-all-bg" onclick="getDevInfo();">GET_DEV_INFO</button>
                            <button class="btn-all btn-main-m btn-all-bg" onclick="getAngTorInfo();">GET_ANGTOR_INFO</button>
                        </div>
                        <div class="sec-3text">
                            <p id="GetInfoText">Device Info / Angle & Torque Info Shows Here.</p>
                        </div>
                    </div>
                </div>
            </section>
            <section class="sec-4">
                <div>
                    <div>
                        <h2 class="tittle-h2">Servo Configration</h2>
                        <div class="Servo-set">
                            <div>
                                <div>
                                    <button class="servo-btn-num btn-num-bg" onmousedown="cmdSend(6,11,0);" ontouchstart="cmdSend(6,11,0);" onmouseup="cmdSend(6,0,0);" ontouchend="cmdSend(6,0,0)">1 -</button>
                                    <button class="servo-btn-num btn-num-bg" onmousedown="cmdSend(6,12,0);" ontouchstart="cmdSend(6,12,0);" onmouseup="cmdSend(6,0,0);" ontouchend="cmdSend(6,0,0)">1 +</button>
                                </div>
                                <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(6,10,0);">SET</button>
                            </div>
                        </div>
                        <div class="Servo-set">
                            <div>
                                <div>
                                    <button class="servo-btn-num btn-num-bg" onmousedown="cmdSend(6,21,0);" ontouchstart="cmdSend(6,21,0);" onmouseup="cmdSend(6,0,0);" ontouchend="cmdSend(6,0,0)">2 -</button>
                                    <button class="servo-btn-num btn-num-bg" onmousedown="cmdSend(6,22,0);" ontouchstart="cmdSend(6,22,0);" onmouseup="cmdSend(6,0,0);" ontouchend="cmdSend(6,0,0)">2 +</button>
                                </div>
                                <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(6,20,0);">SET</button>
                            </div>
                        </div>
                        <div class="Servo-set">
                            <div>
                                <div>
                                    <button class="servo-btn-num btn-num-bg" onmousedown="cmdSend(6,31,0);" ontouchstart="cmdSend(6,31,0);" onmouseup="cmdSend(6,0,0);" ontouchend="cmdSend(6,0,0)">3 -</button>
                                    <button class="servo-btn-num btn-num-bg" onmousedown="cmdSend(6,32,0);" ontouchstart="cmdSend(6,32,0);" onmouseup="cmdSend(6,0,0);" ontouchend="cmdSend(6,0,0)">3 +</button>
                                </div>
                                <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(6,30,0);">SET</button>
                            </div>
                        </div>
                        <div class="Servo-set">
                            <div>
                                <div>
                                    <button class="servo-btn-num btn-num-bg" onmousedown="cmdSend(6,41,0);" ontouchstart="cmdSend(6,41,0);" onmouseup="cmdSend(6,0,0);" ontouchend="cmdSend(6,0,0)">4 -</button>
                                    <button class="servo-btn-num btn-num-bg" onmousedown="cmdSend(6,42,0);" ontouchstart="cmdSend(6,42,0);" onmouseup="cmdSend(6,0,0);" ontouchend="cmdSend(6,0,0)">4 +</button>
                                </div>
                                <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(6,40,0);">SET</button>
                            </div>
                        </div>
                        <div class="Servo-set">
                            <div>
                                <div>
                                    <button class="servo-btn-num btn-num-bg" onmousedown="cmdSend(6,51,0);" ontouchstart="cmdSend(6,51,0);" onmouseup="cmdSend(6,0,0);" ontouchend="cmdSend(6,0,0)">5 -</button>
                                    <button class="servo-btn-num btn-num-bg" onmousedown="cmdSend(6,52,0);" ontouchstart="cmdSend(6,52,0);" onmouseup="cmdSend(6,0,0);" ontouchend="cmdSend(6,0,0)">5 +</button>
                                </div>
                                <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(6,50,0);">SET</button>
                            </div>
                        </div>
                    </div>
                </div>
            </section>
        </div>
    </main>
    <footer>
        <section class="sec-5">
            <button class="btn-stop" onclick="cmdEmergencyStop()">EMERGENCY STOP</button>
            <button id="description-page" class="btn-set" onclick="window.location.href='/help'">
                <svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" fill="none" version="1.1" width="24" height="24" viewBox="0 0 24 24"><g style="mix-blend-mode:passthrough"><g><g style="mix-blend-mode:passthrough"><rect x="0" y="0" width="24" height="24" rx="0" fill="#D8D8D8" fill-opacity="0"/></g><path d="M19.6923,3L15.0769,3C13.8154,3,12.7077,3.615385,12,4.56923C11.29231,3.615385,10.18461,3,8.923079999999999,3L4.30769,3C3.03077,3,2,4.03077,2,5.30769L2,17.6154C2,18.8923,3.03077,19.9231,4.30769,19.9231L8.923079999999999,19.9231C10.2,19.9231,11.23077,20.9538,11.23077,22.2308C11.23077,22.6615,11.56923,23,12,23C12.4308,23,12.7692,22.6615,12.7692,22.2308C12.7692,20.9538,13.8,19.9231,15.0769,19.9231L19.6923,19.9231C20.9692,19.9231,22,18.8923,22,17.6154L22,5.30769C22,4.03077,20.9692,3,19.6923,3ZM11.23077,19.1538C10.58461,18.6769,9.78462,18.3846,8.923079999999999,18.3846L4.30769,18.3846C3.87692,18.3846,3.5384599999999997,18.0462,3.5384599999999997,17.6154L3.5384599999999997,5.30769C3.5384599999999997,4.87692,3.87692,4.53846,4.30769,4.53846L8.923079999999999,4.53846C10.2,4.53846,11.23077,5.56923,11.23077,6.84615L11.23077,19.1538ZM20.4615,17.6154C20.4615,18.0462,20.1231,18.3846,19.6923,18.3846L15.0769,18.3846C14.2154,18.3846,13.4154,18.6769,12.7692,19.1538L12.7692,6.84615C12.7692,5.56923,13.8,4.53846,15.0769,4.53846L19.6923,4.53846C20.1231,4.53846,20.4615,4.87692,20.4615,5.30769L20.4615,17.6154Z" fill="#ffffff" fill-opacity="0.6000000238418579"/></g></g></svg>
            </button>
            <button id="settings-page" class="btn-set" onclick="window.location.href='/config'">
                <svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" fill="none" version="1.1" width="24" height="24" viewBox="0 0 24 24"><clippath id="master_svg0_14_26018/14_25226"><rect x="0" y="0" width="24" height="24" rx="0"/></clippath><g clip-path="url(#master_svg0_14_26018/14_25226)"><g><g style="opacity:0.6000000238418579;"><path d="M21.995596484375,10.1439640625C21.944296484375,9.8761640625,21.736896484375,9.6635040625,21.466696484375,9.6020740625C20.484496484375,9.3792240625,19.667096484375,8.7834140625,19.165496484375,7.9251140625C18.665496484375,7.0682240625,18.551596484375,6.0712140625,18.844596484375,5.1194240625C18.927296484375,4.8587140625,18.844796484375,4.5748440625,18.634796484375,4.3965940625C17.653696484375,3.5593740625,16.536296484375,2.9214790625,15.313896484375,2.5008516625C15.239996484375,2.4753329625,15.162996484375,2.4624297335,15.084696484375,2.4624297335C14.891196484375,2.4624297335,14.705396484375,2.5428471625,14.575496484375,2.6821930625C13.890296484375,3.4097870625,12.962096484375,3.8104840625,11.961996484375,3.8104840625C10.961826484375,3.8104840625,10.032966484375,3.4091160625,9.346186484375,2.6800820625C9.161566484375001,2.4847585625,8.865486484375001,2.4102650625,8.606026484375,2.4989810625C7.375946484375,2.9248840625,6.259196484375,3.5634240625,5.286316484375,4.3971540625C5.079696484375,4.5750840625,4.997506484375,4.857014062499999,5.076916484374999,5.1160340625C5.371916484374999,6.0686740624999995,5.258786484374999,7.0655840625,4.758216484375,7.9234040625C4.258856484375,8.7811340625,3.442206484375,9.376674062500001,2.457268484375,9.6006340625C2.186780484375,9.6641440625,1.979224484375,9.877484062499999,1.928523484375,10.1436040625C1.805990384375,10.7803940625,1.743896484375,11.4173540625,1.743896484375,12.0366640625C1.743896484375,12.6560140625,1.805990384375,13.2930140625,1.928427484375,13.9295140625C1.979392484375,14.1970140625,2.186780484375,14.4097140625,2.457005484375,14.4714140625C3.4393564843749997,14.6944140625,4.256676484374999,15.2899140625,4.758386484375,16.1483140625C5.258766484375,17.0060140625,5.371916484374999,18.0029140625,5.0767464843750005,18.9560140625C4.997916484375001,19.2131140625,5.079606484375001,19.4954140625,5.287066484375,19.6769140625C6.267896484375,20.5138140625,7.384586484375,21.1516140625,8.607276484375,21.5731140625C8.876016484375,21.6642140625,9.163646484375,21.5860140625,9.344486484375,21.3931140625C10.031166484375,20.6644140625,10.959936484375,20.2630140625,11.959996484375,20.2630140625C12.959996484375,20.2630140625,13.888896484375,20.6644140625,14.574696484375,21.3923140625C14.704896484375,21.5319140625,14.890896484375,21.6122140625,15.085196484375,21.6122140625C15.161996484375,21.6122140625,15.238396484375,21.6000140625,15.315596484375,21.5747140625C16.537296484375,21.1535140625,17.653996484375,20.5157140625,18.635596484375,19.6783140625C18.841596484375,19.5007140625,18.923996484375,19.2182140625,18.845096484375,18.9574140625C18.549896484375,18.004014062499998,18.663596484375,17.007214062499997,19.165596484375,16.150214062499998C19.665396484375,15.2918140625,20.482596484375,14.6963140625,21.466996484375,14.4733140625C21.733896484375,14.4125140625,21.941196484375,14.2006140625,21.995496484375,13.9317140625C22.117896484375,13.2946140625,22.179896484375,12.6577140625,22.179896484375,12.0386840625C22.179896484375,11.4183140625,22.117896484375,10.7806840625,21.995596484375,10.1439640625ZM17.075896484375,18.5130140625C16.465896484375,18.9854140625,15.792996484375,19.3697140625,15.072296484375,19.6572140625C14.212596484375,18.9038140625,13.112896484375,18.4905140625,11.959996484375,18.4905140625C10.813296484375,18.4905140625,9.713786484375,18.9038140625,8.848266484375,19.6574140625C8.127696484375,19.3700140625,7.454736484375,18.9857140625,6.844566484375,18.5133140625C7.076466484375,17.401414062500002,6.888836484375,16.2528140625,6.311046484375,15.2642140625C5.738026484375,14.2806140625,4.824466484375,13.5453140625,3.7285464843750002,13.1820140625C3.672736484375,12.7990140625,3.644536484375,12.4142940625,3.644536484375,12.0366940625C3.644536484375,11.6606740625,3.672736484375,11.2761140625,3.7285464843750002,10.8917340625C4.821036484375,10.5310440625,5.734426484375,9.7958140625,6.311046484375,8.809294062500001C6.885816484375,7.8243540625,7.073366484375,6.6757040625,6.844226484375,5.5603840625C7.454156484375,5.0881240625,8.127096484375,4.7039040624999995,8.847996484374999,4.4162640625C9.711006484375,5.1709840625000005,10.810616484375,5.5848940625,11.959996484375,5.5848940625C13.112996484375,5.5848940625,14.212596484375,5.1715640625,15.072296484375,4.4181840625C15.792996484375,4.7057440625,16.465796484374998,5.0899640625,17.075796484374997,5.5622340625C16.845896484375,6.6786540625,17.032696484375002,7.8271140625,17.606996484375,8.8111440625C18.179996484375,9.7947340625,19.093396484375,10.5301240625,20.189396484375,10.8935840625C20.245196484375,11.2776340625,20.273396484375,11.6623540625,20.273396484375,12.0406240625C20.276796484375,12.4025840625,20.249296484375,12.7859140625,20.191396484375,13.1820140625C19.095396484375,13.5454140625,18.181896484375,14.2806140625,17.608896484375002,15.2641140625C17.035396484375,16.2469140625,16.847996484375003,17.3953140625,17.075896484375,18.5130140625Z" fill="#ffffff" fill-opacity="1"/></g><g style="opacity:0.6000000238418579;mix-blend-mode:passthrough"><path d="M16,12C16,14.20914,14.20914,16,12,16C9.79086,16,8,14.20914,8,12C8,9.79086,9.79086,8,12,8C14.20914,8,16,9.79086,16,12ZM14.2,12Q14.2,11.08873,13.55563,10.44437Q12.91127,9.8,12,9.8Q11.08873,9.8,10.44437,10.44437Q9.8,11.08873,9.8,12Q9.8,12.91127,10.44437,13.55563Q11.08873,14.2,12,14.2Q12.91127,14.2,13.55563,13.55563Q14.2,12.91127,14.2,12Z" fill-rule="evenodd" fill="#ffffff" fill-opacity="1"/></g></g></g></svg>
            </button>
        </section>
    </footer>
    <script>
        function cmdEmergencyStop() {
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/stop", true);
            xhr.send();
        }

    	function cmdSend(inputT, inputA, inputB) {
    		var xhr = new XMLHttpRequest();
    		xhr.open("GET", "cmd?inputT="+inputT+"&inputA="+inputA+"&inputB="+inputB, true);
            xhr.send();
    	}

        setInterval(function() {
          getData();
        }, 300);

        function getData() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                  var jsonResponse = JSON.parse(this.responseText);

                  document.getElementById("anglectrl-t1").innerHTML = jsonResponse.A1;
                  document.getElementById("anglectrl-t2").innerHTML = jsonResponse.A2;
                  document.getElementById("anglectrl-t3").innerHTML = jsonResponse.A3;
                  document.getElementById("anglectrl-t4").innerHTML = jsonResponse.A4;
                  document.getElementById("anglectrl-t5").innerHTML = jsonResponse.A5;

                  document.getElementById("coordCtrl-t1").innerHTML = jsonResponse.C1.toFixed(2);
                  document.getElementById("coordCtrl-t2").innerHTML = jsonResponse.C2.toFixed(2);
                  document.getElementById("coordCtrl-t3").innerHTML = jsonResponse.C3.toFixed(2);
                  document.getElementById("coordCtrl-t4").innerHTML = jsonResponse.C4.toFixed(2);
                  document.getElementById("coordCtrl-t5").innerHTML = jsonResponse.C5.toFixed(2);
                }
            };
            xhttp.open("GET", "readData", true);
            xhttp.send();
        }

        function getDevInfo() {
        	var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                  document.getElementById("GetInfoText").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "getDevInfo", true);
            xhttp.send();
        }

        function getAngTorInfo() {
        	var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                  document.getElementById("GetInfoText").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "getAngTorInfo", true);
            xhttp.send();
        }

        function configPage(){
        	cmdSend(8,0,0);
        	var xhr = new XMLHttpRequest();
    		xhr.open("GET", "/config", true);
            xhr.send();
        }
    </script>
</body>
</html>


)rawliteral";



const char setting_html[] PROGMEM = R"rawliteral(


<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>RoArm-M1 Config</title>
<style type="text/css">
    
    body {
        background-image: linear-gradient(#3F424F, #1E212E);
        background-image: -o-linear-gradient(#3F424F, #1E212E);
        background-image: -moz-linear-gradient(#3F424F, #1E212E);
        background-image: -webkit-linear-gradient(#3F424F, #1E212E);
        background-image: -ms-linear-gradient(#3F424F, #1E212E);
        font-family: "roboto",helt "sans-serif";
        font-weight: lighter;
        color: rgba(216,216,216,0.8)
        background-size: cover;
        background-position: center 0;
        background-attachment: fixed;
        color: rgba(255,255,255,0.6);
        border: 0px;
        margin: 0;
        padding: 0;
    }
    main{
        width: 1140px;
        margin: auto;
        margin-bottom: 80px;
    }
    main > div{
        width: 100%;
        display: flex;
        justify-content: space-between;
        padding: 54px 0;
    }
    main > div:first-child{border-bottom: 1px dashed rgba(216,216,216,0.24)}
    section > div{
        width: 516px;
    }
    .btn-all, .btn-of, .btn-num, .btn-num-lr, .servo-btn-num, .btn-stop, .btn-set{cursor: pointer}
    .tittle-h2{
        font-weight:lighter;
        font-size: 24px;
        color: rgba(255,255,255,0.8)
    }
    .sec-1, .sec-3{
        width: 50%;
        border-right: 1px dashed rgba(216,216,216,0.24);
    }
    .set-btn-frame{
        display: flex;
        justify-content: space-between;
        align-items: flex-end;
    }
    .set-btn-frame-i{
        width: 360px;
        display: inline-flex;
        justify-content: space-between;
        text-align: center;
    }
    .set-btn-sections{width: 48px;}
    .btn-num{
        width: 48px;
        height: 48px;
        margin: 1px 0 0 0;
        font-size: 15px;
        font-weight: lighter;
    }
    .btn-num-lr{
        width: 48px;
        height: 46px;
        font-size: 15px;
    }
    .btn-num-up{border-radius: 4px 4px 0 0}
    .btn-num-down{border-radius: 0 0 4px 4px}
    .btn-num-bg{
        background-color: rgba(255,255,255,0.06);
        border: none;
        color: rgba(255,255,255,0.5);
    }
    .btn-num-bg:hover{background-color: rgba(255,255,255,0.02);}
    .btn-all, .btn-of{
        background-color: rgba(164,169,186,0.25);
        border: none;
        border-radius: 4px;
        font-size: 15px;
        color: rgba(255,255,255,0.5);
        font-weight: lighter;
    }
    .btn-of-active{
        background-color: rgba(38,152,234,0.1);
        color: #1EA1FF;
        border: 1px solid #1EA1FF;
        border-radius: 4px;
        font-size: 15px;
        
    }
    .btn-all:hover, .btn-of:hover{background-color:rgba(164,169,186,0.15);}
    .btn-main-l{
        width: 126px;
        height: 97px;
    }
    .init-posit{
        vertical-align: bottom;
    }
    .record-mum-set > div {
        display: flex;
        justify-content: space-between;
        margin-top: 54px;
    }
    .record-mum-set > div > div:last-child {
        width: 320px;
        display: flex;
        justify-content: space-between;
        align-items: flex-start;
    }
    .record-mum-set > div > div:last-child > div {
        display: flex;
        width: 164px;
        border: 1px solid rgba(206,206,206,0.15);
        border-radius: 4px;
    }
    .num-insert{
        background-color: rgba(0,0,0,0.01);
        border: 0px;
        width: 68px;
        text-align: center;
        color: rgba(255,255,255,0.8);
        font-size: 15px;
        font-weight: lighter;
    }
    .btn-main-s{
        width: 126px;
        height: 48px;
    }
    .two-btn > div:first-child{
        display: flex;
        justify-content: space-between;
    }
    .btn-main-m{
        width: 204px;
        height: 48px;
    }
    .two-btn1{
        border-top:1px dashed rgba(216,216,216,0.24) ;
        border-bottom:1px dashed rgba(216,216,216,0.24) ;
        padding: 30px 0;
        margin: 30px 0;
    }
    .Servo-set > div{
        display: flex;
        justify-content: space-between;
    }
    .servo-btn-num{
        height: 48px;
        width: 126px;
        border-radius: 4px;
        font-size: 15px;
        font-weight: lighter;
    }
    .servo-btn-num + button{margin-left: 30px;}
    .Servo-set{margin: 30px 0;}
    .sec-5{
        position: fixed;
        bottom: 0px;
        width: 100%;
        display: flex;
        justify-content: center;
        padding: 40px 0;
        background-image: linear-gradient(rgba(30,33,46,0),rgba(30,33,46,1));
        
    }
    .sec-5 button{margin: 0 15px;}
    .btn-stop{
        width: 204px;
        height: 48px;
        background-color: rgba(181,104,108,1);
        color: white;
        border-radius: 1000px;
        border: none;
    }
    .btn-set{
        width: 48px;
        height: 48px;
        background-color: rgba(115,134,151,1);
        border-radius: 1000px;
        border: none;
    }
    .btn-set:hover{background-color: rgba(115,134,151,0.5);}
    .record-tt{
        color: rgba(255,255,255,0.5);
        font-size: 15px;
        
    }
    .record-height{height: 1px;}
    .sec-3text{
        font-size: 15px;
        text-align: center;
        color: rgba(255,255,255,0.4)
    }
    .btn-stop:hover{background-color: rgba(181,104,108,0.5);}
    input::-webkit-outer-spin-button,input::-webkit-inner-spin-button {
        -webkit-appearance: none;
    }
    input[type='number']{
        -moz-appearance: textfield;
    }
    @media screen and (min-width: 768px) and (max-width: 1200px){
        main{
            width: 516px;
            display: block;
            margin-bottom: 150px;
        }
        .sec-1, .sec-3{width: 100%}
        main > div{display: block;}
        .sec-1, .sec-3{border: none;}
        main > div{padding: 0;}
        main section{
            margin-bottom: 30px;
            border-bottom: 1px dashed rgba(216,216,216,0.24);
            border-top: 1px dashed rgba(216,216,216,0.24);
        }
        .sec-4{border-bottom: none;}
        .record-mum-set > div{margin-top: 30px;}
        main > div:first-child{border: none;}
        .sec-2{padding-bottom: 30px;}
    }
    @media screen and (min-width: 360px) and (max-width: 767px){
        main{
            width: 94%;
            display: block;
        }
        .sec-1, .sec-3{width: 100%}
        main > div{display: block;}
        .sec-1, .sec-3{border: none;}
        section > div{width: auto;}
        .set-btn-frame{
            display: block;
        }
        .set-btn-frame-i{width: 100%;}
        .btn-main-l{
            width: 100%;
            height: 48px;
        }
        .init-posit{
            margin-top: 30px;
        }
        .record-mum-set > div{display: block;}
        .record-mum-set > div > div:last-child{
            width: 100%;
        }
        .sec-5 button{
            margin: 0 4px;
        }
        .two-btn button:first-child{margin-right: 10px;}
        .Servo-set > div > div{
            display: flex;
            width: 66.666666%;
            margin-right: 10px
        }
        .servo-btn-num{
            flex: 1;
        }
        .btn-main-s{width: 33.333%;}
        .servo-btn-num + button{margin-left: 10px;}
        .btn-main-m{
            flex: 1;
            width: auto;
        }
        main > div{padding: 10px 0}
        .record-mum-set > div{margin:  30px 0}
        main section{
            margin-bottom: 30px;
            border-bottom: 1px dashed rgba(216,216,216,0.24);
            border-top: 1px dashed rgba(216,216,216,0.24);
        }
        .sec-4{border-bottom: none;}
        .record-mum-set > div{margin-top: 30px;}
        main > div:first-child{border: none;}
        .record-height{display: none;}
    }
</style>
    <script type="text/javascript">
        function opera(x, y) {
            var rs = new Number(document.getElementById(x).value);

            if (y) {
                document.getElementById(x).value = rs + 0.1;
            } else if( rs >0.1) {
                document.getElementById(x).value = rs - 0.1;
            }
	   }
        function opera1(x, y) {
            var rs = new Number(document.getElementById(x).value);

            if (y) {
                document.getElementById(x).value = rs + 1;
            } else if( rs >-1) {
                document.getElementById(x).value = rs - 1;
            }
	   }
        function opera100(x, y) {
            var rs = new Number(document.getElementById(x).value);

            if (y) {
                document.getElementById(x).value = rs + 100;
            } else if( rs >0) {
                document.getElementById(x).value = rs - 100;
            }
	   }
    </script>
</head>

<body>
    <main>
        <div>
            <section class="sec-1">
                <div>
                    <div>
                        <h2 class="tittle-h2">Make sure there is only ONE servo connected.</h2>
                        <div class="set-btn-frame">
                            <div class="set-btn-frame-i">
                                <div class="set-btn-sections">
                                    <button class="btn-of btn-main-m btn-all-bg" onclick="cmdSend(6,3,0);">Get ID</button>
                                    
                            </div>
                        </div>
                        <p id="config-text">as:</p>
                        <button class="btn-num btn-num-bg btn-num-up" onclick="cmdSend(6,1,1);">1</button>
                        <button class="btn-num btn-num-bg btn-num-up" onclick="cmdSend(6,1,2);">2</button>
                        <button class="btn-num btn-num-bg btn-num-up" onclick="cmdSend(6,1,3);">3</button>
                        <button class="btn-num btn-num-bg btn-num-up" onclick="cmdSend(6,1,4);">4</button>
                        <button class="btn-num btn-num-bg btn-num-up" onclick="cmdSend(6,1,5);">5</button>
                    </div>
                    <div class="two-btn two-btn1">
                        <div>
                            <button class="btn-of btn-main-m btn-all-bg" onclick="cmdSend(6,2,0);">S2 AutoConfig</button>
                        </div>
                    </div>
                    <div>
                        <h2 class="tittle-h2">WebCtrl Config</h2>
                        <div class="record-height"></div>
                        <div class="record-mum-set">
                            <div>
                                <p class="record-tt">AngleCtrl Speed</p>
                                <div>
                                    <div>
                                        <button type="button" onclick="opera('recor-mum1', false);" class="btn-num-lr btn-num-bg btn-num-left">-</button>
                                        <input id="recor-mum1" min="0" value="0.5" type="number" class="num-insert">
                                        <button type="button" onclick="opera('recor-mum1', true);" class="btn-num-lr btn-num-bg btn-num-right">+</button>
                                    </div>
                                    <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(6,4,document.getElementById('recor-mum1').value);">Set</button>
                                </div>
                            </div>
                            <div>
                                <p class="record-tt">CoordCtrl Speed</p>
                                <div>
                                    <div>
                                        <button type="button" onclick="opera('recor-mum2', false);" class="btn-num-lr btn-num-bg btn-num-left">-</button>
                                        <input id="recor-mum2" min="0" value="0.5" type="number" class="num-insert">
                                        <button type="button" onclick="opera('recor-mum2', true);" class="btn-num-lr btn-num-bg btn-num-right">+</button>
                                    </div>
                                    <button class="btn-all btn-main-s btn-all-bg" onclick="cmdSend(6,5,document.getElementById('recor-mum2').value);">Confirm</button>
                                </div>
                            </div>
                        </div>
                    </div>
                    <div class="two-btn two-btn1">
                        <div>
                            <button class="btn-of btn-main-m btn-all-bg" onclick="cmdSend(8,1,0);">ESP-NOW Leader</button>
                            <button class="btn-of btn-main-m btn-all-bg" onclick="cmdSend(8,2,0);">ESP-NOW Follower</button>
                            <button class="btn-of btn-main-m btn-all-bg" onclick="cmdSend(8,3,0);">ESP-NOW OFF</button>
                        </div>
                    </div>
                </div>
            </section>
        </div>
        <div>
            <section class="sec-2">
                <div>
                    <div  class="two-btn two-btn2">
                        <button class="btn-of btn-main-m btn-all-bg" onclick="window.location.href='/'">Main Page</button>
                    </div>
                </div>
            </section>
        </div>
    </main>
    <footer>
        <section class="sec-5">
            <button class="btn-stop" onclick="cmdEmergencyStop()">EMERGENCY STOP</button>
        </section>
    </footer>
    <script>
        function cmdEmergencyStop() {
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/stop", true);
            xhr.send();
        }
    	function cmdSend(inputT, inputA, inputB) {
    		var xhr = new XMLHttpRequest();
    		xhr.open("GET", "config/cmd?inputT="+inputT+"&inputA="+inputA+"&inputB="+inputB, true);
            xhr.send();
    	}

    	function mainPage(){
    		cmdSend(9,0,0);
    		var xhr = new XMLHttpRequest();
    		xhr.open("GET", "/", true);
            xhr.send();
    	}
    </script>
</body>
</html>



)rawliteral";




const char help_html[] PROGMEM = R"rawliteral(


<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>RoArm-M1 Help</title>
<style type="text/css">
    
    body {
        background-image: linear-gradient(#3F424F, #1E212E);
        background-image: -o-linear-gradient(#3F424F, #1E212E);
        background-image: -moz-linear-gradient(#3F424F, #1E212E);
        background-image: -webkit-linear-gradient(#3F424F, #1E212E);
        background-image: -ms-linear-gradient(#3F424F, #1E212E);
        font-family: "roboto",helt "sans-serif";
        font-weight: lighter;
        color: rgba(216,216,216,0.8)
        background-size: cover;
        background-position: center 0;
        background-attachment: fixed;
        color: rgba(255,255,255,0.6);
        border: 0px;
        margin: 0;
        padding: 0;
    }
    main{
        width: 1140px;
        margin: auto;
        margin-bottom: 80px;
    }
    main > div{
        width: 100%;
        display: flex;
        justify-content: space-between;
        padding: 54px 0;
    }
    main > div:first-child{border-bottom: 1px dashed rgba(216,216,216,0.24)}
    section > div{
        width: 516px;
    }
    .btn-all, .btn-of, .btn-num, .btn-num-lr, .servo-btn-num, .btn-stop, .btn-set{cursor: pointer}
    .btn-num-bg:hover{background-color: rgba(255,255,255,0.02);}
    .btn-all, .btn-of{
        background-color: rgba(164,169,186,0.25);
        border: none;
        border-radius: 4px;
        font-size: 15px;
        color: rgba(255,255,255,0.5);
        font-weight: lighter;
    }
    .btn-of-active{
        background-color: rgba(38,152,234,0.1);
        color: #1EA1FF;
        border: 1px solid #1EA1FF;
        border-radius: 4px;
        font-size: 15px;
        
    }
    .btn-all:hover, .btn-of:hover{background-color:rgba(164,169,186,0.15);}
    .btn-main-l{
        width: 126px;
        height: 97px;
    }
    .two-btn > div:first-child{
        display: flex;
        justify-content: space-between;
    }
    .btn-main-m{
        width: 204px;
        height: 48px;
    }
    .two-btn1{
        border-top:1px dashed rgba(216,216,216,0.24) ;
        border-bottom:1px dashed rgba(216,216,216,0.24) ;
        padding: 30px 0;
        margin: 30px 0;
    }
</style>

</head>
<body>
    <main>
        <div>
                <p>
                    EMERGENCY_STOP: {"T":0}<br><br>

                    ANGLE_CTRL: T:cmdType, P1-5:angleInput, <br>S:speedInput, A:accInput<br>
                    {"T":1,"P1":180,"P2":0,"P3":90,"P4":90,"P5":180,"S1":200,"S2":200,"S3":200,"S4":200,"S5":200,"A1":60,"A2":60,"A3":60,"A4":60,"A5":60}<br><br>

                    ANGLE_CTRL_INIT: move to angleCtrl initPos.<br>
                    {"T":11}<br><br>

                    COORD_CTRL: T:cmdType, P1-3:coordInput, P4:thetaAngle, P5:grabberAngle, S1:stepDelay, S5:grabberSpeed<br>
                    {"T":2,"P1":277.5104065,"P2":-13.75,"P3":276.5822754,"P4":90,"P5":180,"S1":10,"S5":200}<br><br>

                    COORD_CTRL_INIT: move to coordCtrl initPos.<br>
                    {"T":12}<br><br>

                    ST_POS_CTRL: T:cmdType, P1-5:posInput, S1-5:speedInput, A1-5:accInput.<br>
                    {"T":3,"P1":2047,"P2":0,"P3":3500,"P4":1500,"P5":2047,"S1":200,"S2":200,"S3":200,"S4":200,"S5":200,"A1":60,"A2":60,"A3":60,"A4":60,"A5":60}<br><br>

                    ST_POS_CTRL_INIT: move to posCtrl initPos.<br>
                    {"T":13}<br><br>

                    --- --- --- --- --- ---<br>
                    GET_DEV_INFO: get device info.<br>
                    {"T":4}<br><br>

                    GET_ANGTOR_INFO: get the angle $ torque info of every servo.<br>
                    {"T":5}<br><br>

                    GET_INFO_BUFFER: get the buffer of IK.<br>
                    {"T":6}<br><br>

                    PLACEHOLDER: Null.<br>
                    {"T":7}<br><br>

                    --- --- === RECORD_REPLAY === --- ---<br>
                    ALL_TORQUE_OFF: {"T":8,"P1":0}<br>
                     ALL_TORQUE_ON: {"T":8,"P1":1}<br>
                     GET_STEP_INFO: {"T":8,"P1":2,"P2":stepNum}<br>
                      MOVE_TO_STEP: {"T":8,"P1":3,"P2":stepNum}<br>
                       REMOVE_STEP: {"T":8,"P1":4,"P2":stepNum}<br>
                        RECORD_POS: {"T":8,"P1":5,"P2":stepNum,"P3":speed(delay every step - 5)}<br>
                      RECORD_DELAY: {"T":8,"P1":15,"P2":stepNum,"P3":delayInput - ms}<br>
                            REPLAY: {"T":8,"P1":6,"P2":stepDelay(delayTime between steps),"P3":loopTime(-1 -forever)}<br>
                    NVS_SPACE_LEFT: {"T":8,"P1":7}<br>
                         NVS_CLEAR: {"T":8,"P1":777}<br><br>


                    --- --- === ROARM_M1_CONFIG === --- ---<br>
                          CONFIG_ALL_INIT: {"T":9,"P1":6} all servos move to pos2047.<br>
                    CONFIG_TORQUE_ALL_OFF: {"T":9,"P1":7}<br>
                     CONFIG_TORQUE_ALL_ON: {"T":9,"P1":8}<br>
                        CONFIG_TORQUE_OFF: {"T":9,"P1":servoNum,"P2":0}<br>
                         CONFIG_TORQUE_ON: {"T":9,"P1":servoNum,"P2":1}<br>
                              CONFIG_MOVE: {"T":9,"P1":servoNum,"P2":2,"P3":PosInput}<br>
                        CONFIG_SET_MIDDLE: {"T":9,"P1":servoNum,"P2":10}<br><br>

                    --- --- === HELP === --- ---<br>
                    HELP:{"T":10}<br>
                </p>
        </div>
        <div>
            <section class="sec-2">
                <div>
                    <div  class="two-btn two-btn2">
                        <button class="btn-of btn-main-m" onclick="window.location.href='/'">Main Page</button>
                    </div>
                </div>
            </section>
        </div>
    </main>
</body>
</html>

)rawliteral";