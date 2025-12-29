<?php
function do_head($title)
{
echo '
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>


/* Setup help text to be shown when hovered on image */

.image-container {
    margin-left: 5px;
    position: relative;
    display: inline-block;
}

.hover-text {
    display: none;
    position: absolute;
    bottom: 10px;
    left: 50%;
    transform: translateX(-50%);
    background-color: rgba(0, 0, 0, 0.7);
    color: white;
    padding: 5px;
    border-radius: 5px;
}

.image-container:hover .hover-text,
.image-container:focus-within .hover-text {
    display: block;
}

img {
    width: 50px; /* Set image size */
    height: auto;
    cursor: pointer;
}


/* The table stuff for admin forms to work in "one form / row" way, which shouldn\'t be done using real tables */

DIV.table 
{
    display:table;
}
FORM.tr, DIV.tr
{
    display:table-row;
}
SPAN.td
{
    display:table-cell;
}

/*
.slider {
  -webkit-appearance: none;
  width: 300px;
  height: 6px;
  border-radius: 3px;
  background: blue;
  outline: none;
}

.slider::-webkit-slider-thumb {
  -webkit-appearance: none;
  appearance: none;
  width: 18px;
  height: 18px;
  border-radius: 50%;
  background: #333;
  cursor: pointer;
}

.slider::-moz-range-thumb {
  width: 18px;
  height: 18px;
  border-radius: 50%;
  background: #333;
  cursor: pointer;
}

*/

.slidecontainer {
  display: flex;
  align-items: center;
  gap: 10px;
  margin: 25px 0;
  font-family: sans-serif;
}

.label {
  width: 30px;
  text-align: center;
}

.slider-wrapper {
  position: relative;
  width: 300px;
}

/* Slider track */
.slider {
  -webkit-appearance: none;
  width: 100%;
  height: 6px;
  border-radius: 3px;
  outline: none;
  background: linear-gradient(to right, red, blue, green);
}

/* Thumb */
.slider::-webkit-slider-thumb {
  -webkit-appearance: none;
  width: 18px;
  height: 18px;
  border-radius: 50%;
  background: #333;
  cursor: pointer;
  position: relative;
  z-index: 2;
}

.slider::-moz-range-thumb {
  width: 18px;
  height: 18px;
  border-radius: 50%;
  background: #333;
  cursor: pointer;
}

/* Floating value bubble */
.value-bubble {
  position: absolute;
  top: -35px;
  transform: translateX(-50%);
  padding: 2px 6px;
  border-radius: 4px;
  background: #333;
  color: #fff;
  font-size: 12px;
  white-space: nowrap;
  pointer-events: none;
}





body {
  background-color: linen;
}

h1 {
  color: maroon;
  margin-left: 40px;
}

table.structure {
  width: 100%;
  border: none;
  text-align: left;
  vertical-align: top;
}
.structure td {
  border: none;
  text-align: left;
  padding-left: 30px;
  padding-right: 30px;
  vertical-align: top;
}
.structure th {
  text-align: left;
  padding-left: 30px;
  padding-right: 30px;
}
.structure {
  border: none;
  text-align: left;
  vertical-align: top;
}

.checkbox-container {
    display: flex;
    align-items: center; /* Align vertically center */
}

.checkboxes {
	/* word-wrap: break-word; */
    margin-right: 10px; /* Space between checkbox and label */
}

table.cardlist {
  width: 100%;
  border: 1px solid black;
  border-radius: 10px;
  border-collapse: collapse;
}

.cardlist th {
  border: 1px solid black;
  padding-top: 12px;
  padding-bottom: 12px;
  text-align: center;
  vertical-align: top;
  background-color: #d2691e;
  color: white;
  border-collapse: collapse;
}

.cardlist th.squeeze {
	width:200px;
}
.cardlist td.squeeze {
	width:200px;
}

.cardlist th.checkbox {
/*	max-width:10px
	text-align: left;
*/
	width:50px;
}
.cardlist td.checkbox {
	width:50px;
/*
	max-width:10px
	text-align: left;
*/
//  overflow: hidden;
 // text-overflow: ellipsis;
 // white-space: nowrap;
}

.cardlist td {
  border: 1px solid black;
  border-collapse: collapse;
  padding-top: 12px;
  padding-bottom: 12px;
  vertical-align: top;
  text-align: center;
  background-color: #ffdead;
  color: brown;
  border-radius: 10px;
}

table.aarvonta {
	border: none;
	width: 100%;
}
.aarvonta th {
  height: 70px;
  text-align: center;
  vertical-align: top;
  background-color: #d2691e;
  color: white;
  border-collapse: collapse;
}

.aarvonta td {
  border: none;
  border-collapse: collapse;
  padding-top: 12px;
  padding-bottom: 12px;
  vertical-align: top;
  text-align: center;
  background-color: ##ffdead;
  color: brown;
  border-radius: 10px;
}

.mandatory {
    border: thin red solid;
}

.help-tip{
/*    position: absolute;
    top: 18px;
    right: 18px; */
    text-align: center;
    background-color: #BCDBEA;
    border-radius: 50%;
    width: 24px;
    height: 24px;
    font-size: 14px;
    line-height: 26px;
    cursor: default;
}

.help-tip:before{
    content:\'?\';
    font-weight: bold;
    color:#fff;
}

.help-tip:hover, .help-tip:focus, .help-tip:active p{
    display:block;
    transform-origin: 100% 0%;

    -webkit-animation: fadeIn 0.3s ease-in-out;
    animation: fadeIn 0.3s ease-in-out;

}

.help-tip p{    /* The tooltip */
    display: none;
    text-align: left;
    background-color: #1E2021;
    padding: 20px;
    width: 300px;
    border-radius: 3px;
    box-shadow: 1px 1px 1px rgba(0, 0, 0, 0.2);
    z-index: 0;
    position:relative;
/*    position: absolute;
    right: -4px; */
    color: #FFF;
    font-size: 13px;
    line-height: 1.4;
}

.help-tip p:before{ /* The pointer of the tooltip */
    position: absolute;
    content: \'\';
    width:0;
    height: 0;
    border:6px solid transparent;
    border-bottom-color:#1E2021;
    left:10px;
    top:-12px;
}

.help-tip p:after{ /* Prevents the tooltip from being hidden */
    width:100%;
    height:40px;
    content:\'\';
    position: absolute;
    top:-40px;
    left:0;
}

/* CSS animation */

@-webkit-keyframes fadeIn {
    0% { 
        opacity:0; 
        transform: scale(0.6);
    }

    100% {
        opacity:100%;
        transform: scale(1);
    }
}

@keyframes fadeIn {
    0% { opacity:0; }
    100% { opacity:100%; }
}

/* For hidden checkboxes */
.hidden {
	display: none;
}

</style>

<script>

/* Toggle checkbox visibility to display expansion specific options (for selected expansions) */
function toggleCheckboxe(checkbox, relatedId) {
	var relatedCheckbox = document.getElementById(relatedId);
	relatedCheckbox.classList.toggle(\'hidden\', !checkbox.checked);
}

function toggleCheckboxes(checkbox, relatedClass) {
	var relatedCheckboxes = document.getElementsByClassName(relatedClass);
	for (var i = 0; i < relatedCheckboxes.length; i++) {
		relatedCheckboxes[i].classList.toggle(\'hidden\', !checkbox.checked);
		if (!checkbox.checked)
			relatedCheckboxes[i].checked = false;
	}
}

function updateSlider(slider) {
  const min = Number(slider.min);
  const max = Number(slider.max);
  const value = Number(slider.value);

  const wrapper = slider.closest(".slider-wrapper");
  const bubble = wrapper.querySelector(".value-bubble");

  // Update bubble text
  bubble.textContent = value;

  // Calculate percentage position
  const percent = ((value - min) / (max - min)) * 100;

  // Position bubble above thumb
  bubble.style.left = `${percent}%`;

  /*
    Continuous gradient:
    - Red at min
    - Blue at zero
    - Green at max
  */
  slider.style.background = `
    linear-gradient(
      to right,
      red 0%,
      blue 50%,
      green 100%
    )
  `;
}

/* Initialize all sliders */
document.addEventListener("DOMContentLoaded", function () {
  document.querySelectorAll(".slider").forEach(slider => {
    updateSlider(slider);
    slider.addEventListener("input", () => updateSlider(slider));
  });
});


/*
function updateSlider(slider) {
  const min = parseInt(slider.min, 10);
  const max = parseInt(slider.max, 10);
  const value = parseInt(slider.value, 10);

  const valueSpan = document.getElementById("sliderValue");
  valueSpan.textContent = value;

  // Normalize value to percentage (0–100)
  const percent = ((value - min) / (max - min)) * 100;
  const center = ((0 - min) / (max - min)) * 100;

  let gradient;

  if (value < 0) {
    gradient = `
      linear-gradient(
        to right,
        red 0%,
        red ${percent}%,
        #ccc ${percent}%,
        #ccc 100%
      )
    `;
  } else if (value > 0) {
    gradient = `
      linear-gradient(
        to right,
        #ccc 0%,
        #ccc ${center}%,
        green ${center}%,
        green ${percent}%,
        #ccc ${percent}%,
        #ccc 100%
      )
    `;
  } else {
    gradient = `
      linear-gradient(
        to right,
        #ccc 0%,
        #ccc ${center - 1}%,
        blue ${center}%,
        #ccc ${center + 1}%,
        #ccc 100%
      )
    `;
  }

  slider.style.background = gradient;
}

document.addEventListener("DOMContentLoaded", function () {
  const slider = document.getElementById("tuhinarange");
  updateSlider(slider);
});

*/

/*
function updateSliderColor(slider) {
  const value = parseInt(slider.value, 10);

  let color;
  if (value < 0) {
    color = "red";
  } else if (value === 0) {
    color = "blue";
  } else {
    color = "green";
  }

  slider.style.background = color;
}

document.addEventListener("DOMContentLoaded", function () {
  const slider = document.getElementById("tuhinarange");
  updateSliderColor(slider);
});
*/
</script>


<title>' . $title . '</title>
</head>
<body>';
}

/* Stolen from the web https://www.geeksforgeeks.org/how-to-detect-a-mobile-device-using-php/ */
function isMobileDevice() { 
    return preg_match("/(android|avantgo|blackberry|bolt|boost|cricket|docomo 
|fone|hiptop|mini|mobi|palm|phone|pie|tablet|up\.browser|up\.link|webos|wos)/i" 
, $_SERVER["HTTP_USER_AGENT"]); 
} 

?>
