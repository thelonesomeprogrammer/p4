import React, { useState, useEffect } from 'react';
import DataInput from './datainput';
import ProgramContainer from './programcontainer';
import PositionGraph from './graphs';

const Main = () => {
	const [programs, setPrograms] = useState([]);
  const [link, setLink] = useState('unknown');
  const [cords, setCords] = useState('unknown');
  const [data, setData] = useState({ x: [], y: [], z: [] });
  const [timestamps, setTimestamps] = useState([]);

  const addProgram = (newProgram) => {
    setPrograms([...programs, newProgram]);
  };


  useEffect(() => {
    const interval = setInterval(async () => {
      try {
        const response = await fetch('http://localhost:8000/api/streams'); // Replace with your API endpoint
        if (!response.ok) throw new Error('Network response was not ok');
				const rawtextdata = await response.text();
				const textdata = rawtextdata.split('link:');
				setCords(textdata[0] || 'unknown');
				setLink(textdata[1] || 'unknown');
				let x = textdata[0].split('x:')[1].split(';')[0];
				let y = textdata[0].split('y:')[1].split(';')[0];
				let z = textdata[0].split('z:')[1].split(';')[0];

        setData((prevData) => ({
          x: [...prevData.x.slice(-50), parseFloat(x)],
          y: [...prevData.y.slice(-50), parseFloat(y)],
          z: [...prevData.z.slice(-50), parseFloat(z)]
        }));

				setTimestamps((prevTimestamps) => [
					...prevTimestamps.slice(-50),
					new Date().toLocaleTimeString()
				]);
      } catch (error) {
        console.error('Fetch error:', error);
      }
    }, 1000); // Update every second

    return () => clearInterval(interval); // Cleanup on component unmount
  }, []);

  return (
    <main>
      <h1>Drone Control Dashboard</h1>
      <div className="main_div">
				<div>
        <DataInput onAddProgram={addProgram} link={link} cords={cords} />
				</div> 
				<div>
        <ProgramContainer programs={programs} setPrograms={setPrograms} />
				</div> 
				<div>
				<PositionGraph data={data} timestamps={timestamps} />
				</div> 
      </div>
    </main>
  );
};

export default Main;
