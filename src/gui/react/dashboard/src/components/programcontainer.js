import React, { useState } from 'react';
import ProgramItem from './programitem';

const ProgramContainer = ({ programs, setPrograms }) => {

	const removeProgram = (index) => {
    setPrograms(programs.filter((_, i) => i !== index));
  };

  const sendProgram = (program) => {
    fetch('/control', { method: 'POST', body: program })
      .then((res) => console.log('Sent:', program))
      .catch((err) => console.error('Error sending program:', err));
  };

  return (
    <div className="program_container">
      <h3>set points</h3>
			<div className="program_div">
				<ul id="list">
					{programs.map((program, index) => (
						<ProgramItem key={index}
												 program={program}
												 onRemove={() => removeProgram(index)}
												 onSend={() => sendProgram(program)}
						/>
					))}
				</ul>
    	</div>
    </div>
  );
};

export default ProgramContainer;
