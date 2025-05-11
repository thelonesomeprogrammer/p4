import React, { useState, useEffect } from "react";
import { Line } from "react-chartjs-2";
import {
	Chart as ChartJS,
	LineElement,
	CategoryScale,
	LinearScale,
	PointElement,
	Title,
	Legend,
	Tooltip,
} from "chart.js";

ChartJS.register(
	LineElement,
	CategoryScale,
	LinearScale,
	PointElement,
	Legend,
	Tooltip,
);

const PositionGraph = ({ data, timestamps }) => {
	const chartData = {
		labels: timestamps,
		datasets: [
			{
				label: "X Position",
				data: data.x,
				borderColor: "rgb(255, 99, 132)",
				backgroundColor: "rgba(255, 99, 132, 0.5)",
				fill: false,
			},
			{
				label: "Y Position",
				data: data.y,
				borderColor: "rgb(54, 162, 235)",
				backgroundColor: "rgba(54, 162, 235, 0.5)",
				fill: false,
			},
			{
				label: "Z Position",
				data: data.z,
				borderColor: "rgb(75, 192, 192)",
				backgroundColor: "rgba(75, 192, 192, 0.5)",
				fill: false,
			},
		],
	};

	const options = {
		responsive: true,
		plugins: {
			legend: {
				position: "top",
			},
		},
		scales: {
			x: {
				title: {
					display: true,
					text: "Time",
				},
			},
			y: {
				title: {
					display: true,
					text: "Position",
				},
			},
		},
	};

	return (
		<div className="graph_div modal">
			<h3>Position Data</h3>
			<Line data={chartData} options={options} />
		</div>
	);
};

export default PositionGraph;
