var model = []

var scaleX = -4 / 64.0;
var scaleY = scaleX * 4;


var centerX = 63 / 2.0;
var centerY = 1.5;

var index = 0;

for (var j = 0; j < 4; j++) {
  for (var i = 0; i < 64; i++) {
      model[index++] = {
          point: [  (i - centerX) * scaleX, 0, (j - centerY) * scaleY ]
      };
  }
}

console.log(JSON.stringify(model));
