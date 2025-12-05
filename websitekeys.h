#pragma once
#include <Arduino.h>

const char Slider[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>

body {
    font-family: Arial, sans-serif;
    background: #1a1a1a;
    color: #eee;
    padding: 20px;
}

h1, h2, h3 {
    color: #00eaff;
    margin-bottom: 5px;
}

.section {
    background: #222;
    padding: 15px;
    border-radius: 10px;
    margin-bottom: 15px;
    box-shadow: 0 0 10px #000;
}

.button {
    background: #0077cc;
    border: none;
    padding: 12px 18px;
    font-size: 16px;
    color: white;
    border-radius: 6px;
    cursor: pointer;
    margin: 5px;
    width: 140px;
}

.button:hover {
    background: #0099ff;
}

.slider {
    width: 100%;
}

.valueBox {
    font-size: 14px;
    margin-top: 4px;
    color: #0f0;
}

#statusBox {
    position: fixed;
    top: 15px;
    right: 15px;
    background: #000;
    color: #0f0;
    padding: 12px;
    border-radius: 8px;
    font-family: monospace;
    font-size: 15px;
    border: 1px solid #0f0;
    width: 190px;
}

</style>
</head>

<body>

<h1>Robot Control Panel</h1>

<div class="section">
    <h2>Speed & Frequency</h2>

    <h3>Frequency (Hz)</h3>
    <input type="range" min="10" max="1000" value="500" class="slider" id="freqBar">
    <div class="valueBox">Value: <span id="freqValue"></span></div>

    <h3>Duty Cycle (%)</h3>
    <input type="range" min="0" max="100" value="100" class="slider" id="dutyBar">
    <div class="valueBox">Value: <span id="dutyValue"></span></div>
</div>

<div class="section">
    <h2>Heading Control</h2>

    <h3>Target Yaw (°)</h3>
    <input type="range" min="-180" max="180" value="0" class="slider" id="yawBar">
    <div class="valueBox">Target: <span id="yawValue"></span>°</div>

    <button class="button" onclick="turnToYaw()">Turn to Target</button>
</div>

<div class="section">
    <h2>Motor Commands</h2>
    <button class="button" onclick="forwardFxn()">Forward</button>
    <button class="button" onclick="backwardFxn()">Backward</button>
    <button class="button" onclick="leftFxn()">Left Pivot</button>
    <button class="button" onclick="rightFxn()">Right Pivot</button>
    <button class="button" style="background:#aa0000" onclick="stopMotor()">STOP</button>
    <button class="button" style="background:#444" onclick="rezero()">Re-Zero Yaw</button>
</div>

<div class="section">
    <h2>PID Tuning</h2>

    <h3>Kp</h3>
    <input type="range" min="0" max="500" value="300" class="slider" id="kpBar">
    <div class="valueBox">Kp: <span id="kpValue"></span></div>

    <h3>Ki</h3>
    <input type="range" min="0" max="100" value="0" class="slider" id="kiBar">
    <div class="valueBox">Ki: <span id="kiValue"></span></div>

    <h3>Kd</h3>
    <input type="range" min="0" max="200" value="0" class="slider" id="kdBar">
    <div class="valueBox">Kd: <span id="kdValue"></span></div>
</div>

<div id="statusBox">
Yaw: <span id="yawVal">0</span>°<br>
Target: <span id="targetVal">0</span>°<br>
LeftPWM: <span id="lpwm">0</span>%<br>
RightPWM: <span id="rpwm">0</span>%
</div>

<script>
// ------- Frequency -------
var freqslider = document.getElementById("freqBar");
var freqoutput = document.getElementById("freqValue");
freqoutput.innerHTML = freqslider.value;
freqslider.oninput = function() {
    freqoutput.innerHTML = this.value;
    fetch(`/FREQ?value=${this.value}`);
}

// ------- Duty -------
var dutyslider = document.getElementById("dutyBar");
var dutyoutput = document.getElementById("dutyValue");
dutyoutput.innerHTML = dutyslider.value;
dutyslider.oninput = function() {
    dutyoutput.innerHTML = this.value;
    fetch(`/DUTY?value=${this.value}`);
}

// ------- Target Yaw -------
var yawslider = document.getElementById("yawBar");
var yawoutput = document.getElementById("yawValue");
yawoutput.innerHTML = yawslider.value;
yawslider.oninput = function() {
    yawoutput.innerHTML = this.value;
    fetch(`/SETYAW?value=${this.value}`);
}

// Turn-to-target with slider auto-reset
function turnToYaw() {
    fetch("/TURNTO")
    .then(r => r.text())
    .then(t => {
        if (t === "DONE") {
            yawslider.value = 0;
            yawoutput.innerHTML = 0;
        }
    });
}

// ------- PID Sliders -------
var kpslider = document.getElementById("kpBar");
var kpoutput = document.getElementById("kpValue");
kpoutput.innerHTML = (kpslider.value/100).toFixed(2);
kpslider.oninput = function() {
    kpoutput.innerHTML = (this.value/100).toFixed(2);
    fetch(`/KP?value=${this.value}`);
}

var kislider = document.getElementById("kiBar");
var kioutput = document.getElementById("kiValue");
kioutput.innerHTML = (kislider.value/100).toFixed(2);
kislider.oninput = function() {
    kioutput.innerHTML = (this.value/100).toFixed(2);
    fetch(`/KI?value=${this.value}`);
}

var kdslider = document.getElementById("kdBar");
var kdoutput = document.getElementById("kdValue");
kdoutput.innerHTML = (kdslider.value/100).toFixed(2);
kdslider.oninput = function() {
    kdoutput.innerHTML = (this.value/100).toFixed(2);
    fetch(`/KD?value=${this.value}`);
}

// ------- Motor Commands -------
function forwardFxn()  { fetch("/FORWARD"); }
function backwardFxn() { fetch("/BACKWARD"); }
function leftFxn()     { fetch("/LEFT"); }
function rightFxn()    { fetch("/RIGHT"); }
function stopMotor()   { fetch("/STOPMOTOR"); }
function rezero()      { fetch("/ZERO"); }

// ------- Status Poll -------
setInterval(() => {
    fetch("/STATUS")
    .then(r => r.text())
    .then(t => {
        let v = t.split(",");
        if (v.length >= 4) {
            document.getElementById("yawVal").innerHTML = v[0];
            document.getElementById("targetVal").innerHTML = v[1];
            document.getElementById("lpwm").innerHTML = v[2];
            document.getElementById("rpwm").innerHTML = v[3];
        }
    });
}, 100);

</script>
</body>
</html>
)===";
