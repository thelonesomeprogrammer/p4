import React, { useState, useEffect } from 'react';

const DataInput = ({ onAddProgram }) => {
  const [link, setLink] = useState('unknown');
  const [cords, setCords] = useState('unknown');
  const [setpoint, setSetpoint] = useState('');

  useEffect(() => {
    const interval = setInterval(async () => {
      try {
        const response = await fetch('/streams');
        if (!response.ok) throw new Error('Network response was not ok');
        const data = await response.text();
        const msg = data.split('link:');
        setLink(msg[1] || 'unknown');
        setCords(msg[2] || 'unknown');
      } catch (error) {
        console.error('Fetch error:', error);
      }
    }, 1000);

    return () => clearInterval(interval); // Cleanup on unmount
  }, []);

  const handleAddSetpoint = () => {
    if (setpoint.trim()) {
      onAddProgram(setpoint.trim());
      setSetpoint(''); // Clear the input field
    } else {
      alert('Setpoint cannot be empty');
    }
  };

  const handleFileChange = (event) => {
    const file = event.target.files[0];
    const reader = new FileReader();
    reader.onload = function () {
      const lines = this.result.split('\n');
      lines.forEach((line) => {
        if (line.trim()) {
          onAddProgram(line.trim());
        }
      });
    };
    reader.readAsText(file);
  };

  return (
    <div className="data_div">
      <p>cf link:</p>
      <p id="link">{link}</p>
      <p>cf position:</p>
      <p id="cords">{cords}</p>
      <p>add setpoint:</p>
      <div className="input_div">
        <input
          name="req"
          id="req"
          type="text"
          value={setpoint}
          onChange={(e) => setSetpoint(e.target.value)}
        />
        <button onClick={handleAddSetpoint}>+</button>
      </div>
      <label htmlFor="file">setpoint save file:</label>
      <input type="file" name="file" id="file" onChange={handleFileChange} />
    </div>
  );
};

export default DataInput;
