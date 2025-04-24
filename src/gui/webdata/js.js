const socket = new WebSocket('ws:localhost:8000/control')
socket.addEventListener('open', (event) => { console.log('open', event.data) })
socket.addEventListener('message', (event) => { console.log('msg', event.data) })
socket.addEventListener('close', (event) => { console.log('close', event.data) })

const evtSource = new EventSource('http:/stream')
evtSource.onmessage = (event) => {
  var msg = event.data
  var msg = msg.split("grip_connect:")
  document.getElementById('grip').textContent = msg[1]
  document.getElementById('cords').textContent = msg[0]
}


function AddToProgramList () {
  var val = document.getElementById('req').value
  AddToProgramList2(val)
  val = ""
}


function AddToProgramList2 (input) {
  var flag = 0
  var points = []
  var speed = 0
  if (input.includes("wait")) {
    let part = input.trim().replace("wait","").replace(":","").replace(";","")
    let value = parseFloat(part.trim());
    if (!isNaN(value)) {
      points.push(value);
    }
    flag = 1
  } else if (input.includes("grip")) {
    let part = input.trim().replace("grip","").replace(":","").replace(";","")
    let value = parseFloat(part.trim());
    if (!isNaN(value)) {
      points.push(value);
    }
    flag = 2
  } else if (input.includes("movj")) {
    let part = input.trim().replace("movj:","")
    let parts = part.split(";")
    if (parts.length < 1) {
      if (parts[1].includes("speed:")) {

      }
    }
    points = parseInput(input)
  } else if (input.includes("movl")) {
    let part = input.trim().replace("movl:","")
    points = parseInput(input)
    flag = 3
  } else {
    points = parseInput(input)
  }
  const newli = document.createElement('li')
  newli.setAttribute("draggable","true")
  const porglist = document.getElementById('list')
  const rmbut = document.createElement('button')
  const pbut = document.createElement('button')
  const newp = document.createElement('p')

  if (points.length == 6) {
    newp.textContent = `t1:${points[0]};t2:${points[1]};t3:${points[2]};t4:${points[3]};t5:${points[4]};t6:${points[5]};`
  } else if (flag == 1 && points.length == 1) {
    newp.textContent = `wait:${points[0]};`
  } else if (flag == 2 && points.length == 1) {
    newp.textContent = `grip:${points[0]};`
  } else {return;}
  rmbut.textContent = "X"
  pbut.textContent = "send"
  pbut.className = "send_button"
  rmbut.className = "rm_button"
  rmbut.onclick = function(){removeli(rmbut)}
  pbut.onclick = function(){senditbro(pbut)}
  newli.appendChild(newp)
  newli.appendChild(pbut)
  newli.appendChild(rmbut)
  porglist.appendChild(newli)
}

function senditbro(but){
  const li = but.parentNode
    if (li.firstChild !=null) {
      const p = li.firstChild
      console.log(p.textContent)
      socket.send(p.textContent)
  }
}

function removeli(but) {
  const li = but.parentNode
  const ul = li.parentNode
  ul.removeChild(li)
}

document.getElementById('file').onchange = function() {
  var file = this.files[0];

  var reader = new FileReader();
  reader.onload = function(progressEvent) {
    const text = this.result;
    var lines = text.split('\n');
    for (var line = 0; line < lines.length; line++) {
      AddToProgramList2(lines[line])
    }
  };
  reader.readAsText(file);
};


function flush() {
  const porglist = document.getElementById('list')
  porglist.childNodes.forEach(li => {
    if (li.firstChild !=null) {
      const p = li.firstChild
      console.log(p.textContent)
      socket.send(p.textContent)
  }})
}

function stopbot () {
  socket.send("stop")
}

function parseInput(input) {
  let values = [];

  if (input.includes('\n')){
    inputlist = input.split("\n")
  } else {
    inputlist = [input]
  }
    
  inputlist.forEach(input => {
    if (input.includes(':')) {
      let parts = input.split(/,|;/);

      parts.forEach(part => {
        let pair = part.split(':');
        if (pair.length === 2) {
          let value = parseFloat(pair[1].trim());
          if (!isNaN(value)) {
            values.push(value.toFixed(2));
          }
        }
      });
    } else { // No ':', split by ','
      let parts = input.split(',');
        parts.forEach(part => {
          let value = parseFloat(part.trim());
            if (!isNaN(value)) {
              values.push(value.toFixed(2));
            }
        });
    }
  })
  return values;
}

var dragging = null;

document.addEventListener('dragstart', function(event) {
    var target = getLI( event.target );
    dragging = target;
    event.dataTransfer.setData('text/plain', null);
    event.dataTransfer.setDragImage(self.dragging,0,0);
});

document.addEventListener('dragover', function(event) {
    event.preventDefault();
    var target = getLI( event.target );
    if (target == false) {
      return
    }
    var bounding = target.getBoundingClientRect()
    var offset = bounding.y + (bounding.height/2);
    if ( offset < event.y ) {
       	target.style['border-bottom-color'] = 'blue';
        target.style['border-top-color'] = '#f2f2f2';
    } else {
        target.style['border-top-color'] = 'blue';
        target.style['border-bottom-color'] = '#f2f2f2';
    }
});

document.addEventListener('dragleave', function(event) {
    var target = getLI( event.target );
    if (target == false) {
      return
    }
    target.style['border-bottom-color'] = '#f2f2f2';
    target.style['border-top-color'] = '#f2f2f2';
});

document.addEventListener('drop', function(event) {
    event.preventDefault();
    var target = getLI( event.target );
    if (target == false) {
      return
    }
    target.style['border-bottom-color'] = '#f2f2f2';
    target.style['border-top-color'] = '#f2f2f2';
    var bounding = target.getBoundingClientRect()
    var offset = bounding.y + (bounding.height/2);
    if ( offset < event.y) {
        target.parentNode.insertBefore(dragging, target.nextSibling);
    } else {
        target.parentNode.insertBefore(dragging, target);
    }
});

function getLI( target ) {
    while ( target.nodeName.toLowerCase() != 'li' && target.nodeName.toLowerCase() != 'body' ) {
        target = target.parentNode;
    }
    if ( target.nodeName.toLowerCase() == 'body' ) {
        return false;
    } else {
        return target;
    }
}
