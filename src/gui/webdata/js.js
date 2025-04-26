setInterval(async () => {
	try {
		const response = await fetch('/streams');
		if (!response.ok) {
			throw new Error('Network response was not ok');
		}
		const data = await response.text();
		var msg = data.split("link:")
		document.getElementById('link').textContent = msg[1]
		document.getElementById('cords').textContent = msg[0]
		console.log(data);
	} catch (error) {
		console.error('Fetch error:', error);
	}
}, 1000);




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
  } else if (input.includes("mov")) {
    let part = input.trim().replace("movl:","")
    points = parseInput(input)
    flag = 2
  } else {
    points = parseInput(input)
  }
  const newli = document.createElement('li')
  newli.setAttribute("draggable","true")
  const porglist = document.getElementById('list')
  const rmbut = document.createElement('button')
  const pbut = document.createElement('button')
  const newp = document.createElement('p')

  if (points.length == 3) {
    newp.textContent = `x:${points[0]};y:${points[1]};z:${points[2]};`
  } else if (flag == 1 && points.length == 1) {
    newp.textContent = `wait:${points[0]};`
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

async function senditbro(but){
  const li = but.parentNode
    if (li.firstChild !=null) {
      const p = li.firstChild
      console.log(p.textContent)
			const response = await fetch('/control', { method: 'POST', body: p.textContent});
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
