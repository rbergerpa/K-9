function initDrive() {
  $('.btn-steering').on('mousedown', function() {
    putJSON('/steering', { steering: $(this).data('steering') })
  });

  // $('.btn-steering').on('mouseup', function() {
  //   putJSON('/steering', { steering: 0 })
  // });

  $('.btn-speed').on('mousedown', function() {
    putJSON('/speed', { speed: $(this).data('speed') })
  });

  $('a.sound').on('mousedown', function() {
    console.log("sound " + $(this).data('filename'));
    putJSON('/sound', { filename: $(this).data('filename') })
  });


  console.log('webcam: ' + 'http://' + window.location.hostname + ':8080/?action=stream');
  $('#webcam').attr('src', 'http://' + window.location.hostname + ':8080/?action=stream');

}

function putJSON(url, data) {
console.log("url: " + url);
console.log("data: " + JSON.stringify(data));

  $.ajax({
    url: url,
    type: 'PUT',
    contentType: 'application/json',
    dataType: 'json',
    data: JSON.stringify(data)
  });
}

$(document).ready(function() {
  initDrive();
  $('.menu').dropit();
});

