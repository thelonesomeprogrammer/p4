import React from 'react';
import React, { useState } from 'react';
import DataInput from './datainput';
import ProgramContainer from './programcontainer';

const Main = () => {
	const [programs, setPrograms] = useState([]);

  const addProgram = (newProgram) => {
    setPrograms([...programs, newProgram]);
  };

  return (
    <main>
      <h1>Drone Control Dashboard</h1>
      <div className="main_div">
        <DataInput onAddProgram={addProgram} />
        <ProgramContainer programs={programs} setPrograms={setPrograms} />
      </div>
    </main>
  );
};

export default Main;
