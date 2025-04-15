// Add these functions at the beginning of your script.js file

// Mathematical expression evaluator
function evaluateExpression(expr) {
    if (!expr || typeof expr !== 'string') return NaN;

    const processedExpr = expr.trim()
        .replace(/sqrt\(/g, 'Math.sqrt(')
        .replace(/\^/g, '**')
        .replace(/(\d+|\))\s*\(/g, '$1*('); // Implicit multiplication

    try {
        return Function('"use strict"; return (' + processedExpr + ')')();
    } catch (e) {
        console.error("Error evaluating expression:", expr, e);
        throw new Error(`Cannot evaluate "${expr}"`);
    }
}


function parseNodes(input) {
    if (!input.trim()) return [];

    return input.split(";").map((pair, index) => {
        const coords = pair.trim().split(",");
        if (coords.length !== 2) {
            throw new Error(`Invalid node format at position ${index + 1}`);
        }

        // Helper to clean up outer parentheses, if any
        const clean = str => str.trim().replace(/^\((.*)\)$/, '$1');

        try {
            const x = evaluateExpression(clean(coords[0]));
            const y = evaluateExpression(clean(coords[1]));

            if (isNaN(x) || isNaN(y)) {
                throw new Error(`Invalid numeric value`);
            }

            return [x, y];
        } catch (error) {
            throw new Error(`Error in coordinates (${coords[0]},${coords[1]}): ${error.message}`);
        }
    });
}




function parseLoads(input) {
    if (!input.trim()) return [];
    
    return input.split(";").map(load => {
        const parts = load.trim().split(",");
        if (parts.length !== 3) {
            throw new Error("Invalid load format. Use node,fx,fy");
        }
        
        try {
            const node = parseInt(parts[0]);
            const fx = evaluateExpression(parts[1]);
            const fy = evaluateExpression(parts[2]);
            
            if (isNaN(node) || isNaN(fx) || isNaN(fy)) {
                throw new Error("Invalid load values");
            }
            
            return { node, fx, fy };
        } catch (error) {
            throw new Error(`Error in load expression: ${error.message}`);
        }
    });
}


// Add angle-based node positioning
function showAngleNodeForm() {
    const modal = document.createElement('div');
    modal.className = 'modal';
    modal.innerHTML = `
        <div class="modal-content">
            <h3>Add Node by Angle</h3>
            <div class="error-message" style="color:red; display:none;"></div>
            <div class="form-group">
                <label>Reference Point:</label>
                <select id="reference-node">${getNodeOptions()}</select>
            </div>
            <div class="form-group">
                <label>Distance:</label>
                <input type="text" id="node-distance" placeholder="e.g., 5 or 2*sqrt(3)">
            </div>
            <div class="form-group">
                <label>Angle (degrees):</label>
                <input type="text" id="node-angle" placeholder="e.g., 60">
            </div>
            <div class="button-group">
                <button id="add-angle-node">Add Node</button>
                <button id="cancel-angle-node">Cancel</button>
            </div>
        </div>
    `;
    
    document.body.appendChild(modal);
    const errorDiv = modal.querySelector('.error-message');

    document.getElementById('cancel-angle-node').addEventListener('click', () => {
        document.body.removeChild(modal);
    });
    
    document.getElementById('add-angle-node').addEventListener('click', () => {
        errorDiv.style.display = 'none';
        
        try {
            const reference = document.getElementById('reference-node').value;
            const distance = evaluateExpression(document.getElementById('node-distance').value);
            const angle = evaluateExpression(document.getElementById('node-angle').value);
            
            if (isNaN(distance) || isNaN(angle)) {
                throw new Error("Invalid numeric values");
            }
            
            let baseX = 0, baseY = 0;
            if (reference !== 'origin') {
                const nodes = parseNodes(document.getElementById('nodes').value);
                const nodeIndex = parseInt(reference);
                if (nodeIndex >= nodes.length) throw new Error("Invalid reference node");
                [baseX, baseY] = nodes[nodeIndex];
            }
            
            const angleRad = angle * Math.PI / 180;
            const newX = baseX + distance * Math.cos(angleRad);
            const newY = baseY + distance * Math.sin(angleRad);
            
            const nodesTextarea = document.getElementById('nodes');
            nodesTextarea.value = nodesTextarea.value + 
                (nodesTextarea.value ? '; ' : '') +
                `${newX.toFixed(6)},${newY.toFixed(6)}`;
                
            document.body.removeChild(modal);
        } catch (error) {
            errorDiv.textContent = "Error: " + error.message;
            errorDiv.style.display = 'block';
        }
    });
}


// Helper to generate options for existing nodes
function getNodeOptions() {
    try {
        const nodesText = document.getElementById('nodes').value;
        if (!nodesText.trim()) return '';
        
        const nodes = parseNodes(nodesText);
        return nodes.map((node, index) => 
            `<option value="${index}">Node ${index}: (${node[0].toFixed(2)}, ${node[1].toFixed(2)})</option>`
        ).join('');
    } catch (e) {
        return '';
    }
}


// Update document ready function to add angle button
document.addEventListener('DOMContentLoaded', function() {
    document.getElementById("analyze-btn").addEventListener("click", analyzeTruss);
    
    // Add button for angle-based node positioning
    const nodesGroup = document.querySelector('.input-group');
    const angleButton = document.createElement('button');
    angleButton.type = 'button';
    angleButton.id = 'add-angle-btn';
    angleButton.className = 'secondary-btn';
    angleButton.textContent = 'Add Node by Angle';
    angleButton.addEventListener('click', showAngleNodeForm);
    
    nodesGroup.appendChild(angleButton);
});




// Direct Stiffness Method for Truss Analysis
document.addEventListener('DOMContentLoaded', function() {
    document.getElementById("analyze-btn").addEventListener("click", analyzeTruss);
    
    // Initialize canvases
    clearCanvases();
});

function clearCanvases() {
    const inputCanvas = document.getElementById("input-canvas");
    const resultsCanvas = document.getElementById("results-canvas");
    const ctxInput = inputCanvas.getContext("2d");
    const ctxResults = resultsCanvas.getContext("2d");
    
    ctxInput.clearRect(0, 0, inputCanvas.width, inputCanvas.height);
    ctxResults.clearRect(0, 0, resultsCanvas.width, resultsCanvas.height);
    
    // Add placeholder text
    ctxInput.fillStyle = "#999";
    ctxInput.font = "16px Arial";
    ctxInput.textAlign = "center";
    ctxInput.fillText("Input visualization will appear here after analysis", inputCanvas.width/2, inputCanvas.height/2);
    
    ctxResults.fillStyle = "#999";
    ctxResults.font = "16px Arial";
    ctxResults.textAlign = "center";
    ctxResults.fillText("Results visualization will appear here after analysis", resultsCanvas.width/2, resultsCanvas.height/2);
}

function analyzeTruss() {
    try {
        // Get user input
        const nodesInput = document.getElementById("nodes").value.trim();
        const elementsInput = document.getElementById("elements").value.trim();
        const supportsInput = document.getElementById("supports").value.trim();
        const loadsInput = document.getElementById("loads").value.trim();
        
        // Validate inputs
        if (!nodesInput || !elementsInput || !supportsInput || !loadsInput) {
            throw new Error("All input fields are required");
        }
        
        // Parse input values
        const nodes = parseNodes(nodesInput);
        const elements = parseElements(elementsInput);
        const supports = parseSupports(supportsInput);
        const loads = parseLoads(loadsInput);
        
        // Validate parsed data
        if (nodes.length < 2) throw new Error("At least 2 nodes are required");
        if (elements.length < 1) throw new Error("At least 1 element is required");
        if (supports.length < 2) throw new Error("At least 2 supports are required for stability");
        
        // Check node indices in elements
        elements.forEach(([i, j]) => {
            if (i < 0 || i >= nodes.length || j < 0 || j >= nodes.length) {
                throw new Error(`Element references node index out of range: ${i}-${j}`);
            }
        });
        
        // Check node indices in supports and loads
        supports.forEach(support => {
            if (support.node < 0 || support.node >= nodes.length) {
                throw new Error(`Support references node index out of range: ${support.node}`);
            }
        });
        
        loads.forEach(load => {
            if (load.node < 0 || load.node >= nodes.length) {
                throw new Error(`Load references node index out of range: ${load.node}`);
            }
        });

        // Perform analysis using direct stiffness method
        const results = directStiffnessMethod(nodes, elements, supports, loads);

        // Display results
        displayResults(results, elements);

        // Visualize input configuration
        visualizeInput(nodes, elements, supports, loads);
        
        // Visualize results
        visualizeResults(nodes, elements, results);
    } catch (error) {
        alert("Error in analysis: " + error.message);
        console.error(error);
        clearCanvases();
        document.getElementById("results").innerHTML = `<p class="error">Error: ${error.message}</p>`;
    }
}

// Parser functions
function parseNodes(input) {
    return input.split(";").map(pair => {
        const coords = pair.trim().split(",").map(Number);
        if (coords.length !== 2 || coords.some(isNaN)) {
            throw new Error("Invalid node coordinates format. Use x,y; x,y; ...");
        }
        return coords;
    });
}

function parseElements(input) {
    return input.split(";").map(pair => {
        const nodes = pair.trim().split("-").map(Number);
        if (nodes.length !== 2 || nodes.some(isNaN)) {
            throw new Error("Invalid element format. Use node1-node2; node3-node4; ...");
        }
        return nodes;
    });
}

function parseSupports(input) {
    return input.split(";").map(support => {
        const parts = support.trim().split(",");
        if (parts.length < 2) {
            throw new Error("Invalid support format. Use node,type,direction");
        }
        
        const node = parseInt(parts[0]);
        if (isNaN(node)) {
            throw new Error("Invalid node number in support definition");
        }
        
        const type = parts[1].trim();
        if (type !== 'pin' && type !== 'roller') {
            throw new Error("Support type must be 'pin' or 'roller'");
        }
        
        let direction = null;
        if (type === 'roller') {
            if (parts.length < 3) {
                throw new Error("Roller supports require a direction (x or y)");
            }
            direction = parts[2].trim();
            if (direction !== 'x' && direction !== 'y') {
                throw new Error("Roller direction must be 'x' or 'y'");
            }
        }
        
        return { node, type, direction };
    });
}

function parseLoads(input) {
    return input.split(";").map(load => {
        const parts = load.trim().split(",").map(Number);
        if (parts.length !== 3 || parts.some(isNaN)) {
            throw new Error("Invalid load format. Use node,fx,fy");
        }
        return { node: parts[0], fx: parts[1], fy: parts[2] };
    });
}

// Direct Stiffness Method Implementation
function directStiffnessMethod(nodes, elements, supports, loads) {
    // Constants
    const E = 200e9;  // Young's modulus (Pa)
    const A = 0.01;   // Cross-sectional area (m²)
    
    const numNodes = nodes.length;
    const numDOF = 2 * numNodes;
    
    // Initialize global stiffness matrix
    let K = [];
    for (let i = 0; i < numDOF; i++) {
        K[i] = Array(numDOF).fill(0);
    }
    
    // Assemble global stiffness matrix
    elements.forEach(([i, j]) => {
        // Calculate element length and orientation
        const [xi, yi] = nodes[i];
        const [xj, yj] = nodes[j];
        
        const dx = xj - xi;
        const dy = yj - yi;
        const L = Math.sqrt(dx*dx + dy*dy);
        
        // Direction cosines
        const c = dx/L;
        const s = dy/L;
        
        // Element stiffness matrix in global coordinates
        const k = [
            [(E*A/L) * c*c, (E*A/L) * c*s, -(E*A/L) * c*c, -(E*A/L) * c*s],
            [(E*A/L) * c*s, (E*A/L) * s*s, -(E*A/L) * c*s, -(E*A/L) * s*s],
            [-(E*A/L) * c*c, -(E*A/L) * c*s, (E*A/L) * c*c, (E*A/L) * c*s],
            [-(E*A/L) * c*s, -(E*A/L) * s*s, (E*A/L) * c*s, (E*A/L) * s*s]
        ];
        
        // Assemble into global stiffness matrix
        const dofs = [2*i, 2*i+1, 2*j, 2*j+1];
        for (let m = 0; m < 4; m++) {
            for (let n = 0; n < 4; n++) {
                K[dofs[m]][dofs[n]] += k[m][n];
            }
        }
    });
    
    // Apply boundary conditions
    let fixedDOFs = [];
    supports.forEach(support => {
        if (support.type === 'pin') {
            fixedDOFs.push(2 * support.node);     // x-direction
            fixedDOFs.push(2 * support.node + 1); // y-direction
        } else if (support.type === 'roller') {
            if (support.direction === 'x') {
                fixedDOFs.push(2 * support.node);     // constrained in x
            } else {
                fixedDOFs.push(2 * support.node + 1); // constrained in y
            }
        }
    });
    
    // Sort and deduplicate fixed DOFs
    fixedDOFs = [...new Set(fixedDOFs)].sort((a, b) => a - b);
    
    // Create list of free DOFs
    const freeDOFs = [];
    for (let i = 0; i < numDOF; i++) {
        if (!fixedDOFs.includes(i)) {
            freeDOFs.push(i);
        }
    }
    
    // Create force vector
    let F = Array(numDOF).fill(0);
    loads.forEach(load => {
        if (load.fx) F[2 * load.node] += load.fx;
        if (load.fy) F[2 * load.node + 1] += load.fy;
    });
    
    // Extract submatrices for free DOFs
    const Kff = [];
    for (let i = 0; i < freeDOFs.length; i++) {
        Kff[i] = [];
        for (let j = 0; j < freeDOFs.length; j++) {
            Kff[i][j] = K[freeDOFs[i]][freeDOFs[j]];
        }
    }
    
    const Ff = freeDOFs.map(dof => F[dof]);
    
    // Solve for displacements using matrix inversion
    const Uf = solveSystem(Kff, Ff);
    
    // Full displacement vector
    let U = Array(numDOF).fill(0);
    for (let i = 0; i < freeDOFs.length; i++) {
        U[freeDOFs[i]] = Uf[i];
    }
    
    // Calculate member forces
    const forces = elements.map(([i, j]) => {
        const [xi, yi] = nodes[i];
        const [xj, yj] = nodes[j];
        
        const dx = xj - xi;
        const dy = yj - yi;
        const L = Math.sqrt(dx*dx + dy*dy);
        
        const c = dx/L;
        const s = dy/L;
        
        // Transformation for axial force
        const force = (E*A/L) * ((U[2*j]*c + U[2*j+1]*s) - (U[2*i]*c + U[2*i+1]*s));
        return force;
    });
    
    return forces;
}

// Matrix solver function (Gaussian elimination)
function solveSystem(A, b) {
    const n = b.length;
    
    // Create augmented matrix
    const augmented = [];
    for (let i = 0; i < n; i++) {
        augmented[i] = [...A[i], b[i]];
    }
    
    // Gaussian elimination
    for (let i = 0; i < n; i++) {
        // Find maximum element in column
        let maxRow = i;
        for (let j = i + 1; j < n; j++) {
            if (Math.abs(augmented[j][i]) > Math.abs(augmented[maxRow][i])) {
                maxRow = j;
            }
        }
        
        // Swap rows
        [augmented[i], augmented[maxRow]] = [augmented[maxRow], augmented[i]];
        
        // Zero out below pivot
        for (let j = i + 1; j < n; j++) {
            const factor = augmented[j][i] / augmented[i][i];
            for (let k = i; k <= n; k++) {
                augmented[j][k] -= factor * augmented[i][k];
            }
        }
    }
    
    // Back substitution
    const x = Array(n).fill(0);
    for (let i = n - 1; i >= 0; i--) {
        x[i] = augmented[i][n];
        for (let j = i + 1; j < n; j++) {
            x[i] -= augmented[i][j] * x[j];
        }
        x[i] /= augmented[i][i];
    }
    
    return x;
}

function displayResults(forces, elements) {
    const resultsDiv = document.getElementById("results");
    resultsDiv.innerHTML = "";
    
    if (forces.length === 0) {
        resultsDiv.innerHTML = "<p>No results to display.</p>";
        return;
    }
    
    forces.forEach((force, i) => {
        const [node1, node2] = elements[i];
        const div = document.createElement("div");
        div.className = `result-card ${force > 0 ? 'tension' : 'compression'}`;
        div.innerHTML = `
            <strong>Member ${i+1} (${node1}-${node2}):</strong><br>
            ${Math.abs(force/1000).toFixed(5)} kN ${force > 0 ? 'Tension' : 'Compression'}
        `;
        resultsDiv.appendChild(div);
    });
}



function visualizeInput(nodes, elements, supports, loads) {
    const canvas = document.getElementById("input-canvas");
    const ctx = canvas.getContext("2d");

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Find boundaries
    let minX = Math.min(...nodes.map(n => n[0]));
    let maxX = Math.max(...nodes.map(n => n[0]));
    let minY = Math.min(...nodes.map(n => n[1]));
    let maxY = Math.max(...nodes.map(n => n[1]));

    const padding = 50;
    const rangeX = maxX - minX;
    const rangeY = maxY - minY;

    const scaleX = rangeX === 0 ? 1 : (canvas.width - 2 * padding) / rangeX;
    const scaleY = rangeY === 0 ? 1 : (canvas.height - 2 * padding) / rangeY;
    const scale = Math.min(scaleX, scaleY);

    const offsetX = padding + (canvas.width - 2 * padding - scale * rangeX) / 2;
    const offsetY = padding + (canvas.height - 2 * padding - scale * rangeY) / 2;

    const transform = (x, y) => [
        offsetX + scale * (x - minX),
        canvas.height - (offsetY + scale * (y - minY)) // Invert Y
    ];

    // Draw elements
    ctx.strokeStyle = 'black';
    ctx.lineWidth = 2;
    elements.forEach(([i, j], index) => {
        const [x1, y1] = transform(...nodes[i]);
        const [x2, y2] = transform(...nodes[j]);

        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();

        const midX = (x1 + x2) / 2;
        const midY = (y1 + y2) / 2;
        ctx.fillStyle = 'blue';
        ctx.font = '14px Arial';
        ctx.fillText(`(${index + 1})`, midX + 5, midY - 5);
    });

    // Draw nodes
    ctx.fillStyle = 'black';
    nodes.forEach(([x, y], i) => {
        const [tx, ty] = transform(x, y);
        ctx.beginPath();
        ctx.arc(tx, ty, 6, 0, 2 * Math.PI);
        ctx.fill();
        ctx.font = '16px Arial';
        ctx.fillText(String.fromCharCode(65 + i), tx + 10, ty + 5);
    });

    // Draw supports
    supports.forEach(support => {
        const [x, y] = transform(...nodes[support.node]);

        if (support.type === 'pin') {
            // Adjust triangle to touch the node
            const size = 20;
            ctx.fillStyle = 'gray';
            ctx.beginPath();
            ctx.moveTo(x, y); // Top of triangle at node
            ctx.lineTo(x - size, y + size);
            ctx.lineTo(x + size, y + size);
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
        } else if (support.type === 'roller') {
            const size = 8;
            ctx.fillStyle = 'gray';
            ctx.beginPath();
            ctx.arc(x, y + size + 4, size, 0, 2 * Math.PI);
            ctx.fill();
            ctx.stroke();
            ctx.beginPath();
            ctx.moveTo(x - size - 4, y + size + 12);
            ctx.lineTo(x + size + 4, y + size + 12);
            ctx.stroke();
        }
    });

    // Draw loads
    loads.forEach(load => {
        const [x, y] = transform(...nodes[load.node]);
        const arrowSize = 10;

        ctx.strokeStyle = 'red';
        ctx.fillStyle = 'red';
        ctx.lineWidth = 2;

        // Horizontal (Fx)
        if (load.fx !== 0) {
            const direction = Math.sign(load.fx);
            const magnitude = Math.abs(load.fx);

            ctx.beginPath();
            ctx.moveTo(x, y);
            ctx.lineTo(x + direction * 40, y);
            ctx.stroke();

            // Arrowhead
            ctx.beginPath();
            ctx.moveTo(x + direction * 40, y);
            ctx.lineTo(x + direction * (40 - arrowSize), y - arrowSize / 2);
            ctx.lineTo(x + direction * (40 - arrowSize), y + arrowSize / 2);
            ctx.closePath();
            ctx.fill();

            // Label
            ctx.font = '14px Arial';
            ctx.fillText(`${magnitude} N`, x + direction * 20, y - 10);
        }

        // Vertical (Fy)
        if (load.fy !== 0) {
            const direction = -Math.sign(load.fy);  // Invert for canvas Y
            const magnitude = Math.abs(load.fy);

            ctx.beginPath();
            ctx.moveTo(x, y);
            ctx.lineTo(x, y + direction * 40);
            ctx.stroke();

            // Arrowhead
            ctx.beginPath();
            ctx.moveTo(x, y + direction * 40);
            ctx.lineTo(x - arrowSize / 2, y + direction * (40 - arrowSize));
            ctx.lineTo(x + arrowSize / 2, y + direction * (40 - arrowSize));
            ctx.closePath();
            ctx.fill();

            // Label
            ctx.font = '14px Arial';
            ctx.fillText(`${magnitude} N`, x + 10, y + direction * 20);
        }
    });
}



function visualizeResults(nodes, elements, forces) {
    const canvas = document.getElementById("results-canvas");
    const ctx = canvas.getContext("2d");

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Find boundaries
    let minX = Math.min(...nodes.map(n => n[0]));
    let maxX = Math.max(...nodes.map(n => n[0]));
    let minY = Math.min(...nodes.map(n => n[1]));
    let maxY = Math.max(...nodes.map(n => n[1]));

    // Add padding
    const padding = 50;

    // Calculate scale and offset
    const rangeX = maxX - minX;
    const rangeY = maxY - minY;
    const scaleX = rangeX === 0 ? 1 : (canvas.width - 2 * padding) / rangeX;
    const scaleY = rangeY === 0 ? 1 : (canvas.height - 2 * padding) / rangeY;
    const scale = Math.min(scaleX, scaleY);
    const offsetX = padding + (canvas.width - 2 * padding - scale * rangeX) / 2;
    const offsetY = padding + (canvas.height - 2 * padding - scale * rangeY) / 2;

    // Transform function
    const transform = (x, y) => [
        offsetX + scale * (x - minX),
        canvas.height - (offsetY + scale * (y - minY))
    ];

    // Find maximum force
    const maxForce = Math.max(...forces.map(f => Math.abs(f)));

    // Draw elements
    elements.forEach(([i, j], index) => {
        const [x1, y1] = transform(...nodes[i]);
        const [x2, y2] = transform(...nodes[j]);
        const force = forces[index];

        // Color for tension/compression
        ctx.strokeStyle = force > 0 ? 'blue' : 'red';

        // Line width based on magnitude
        const relativeForce = Math.abs(force) / maxForce;
        ctx.lineWidth = 2 + relativeForce * 8;

        // Draw element line
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();

        // Draw force label in black, above the edge
        const midX = (x1 + x2) / 2;
        const midY = (y1 + y2) / 2;
        ctx.fillStyle = 'black';
        ctx.font = 'bold 14px Arial';
        ctx.fillText(`${Math.abs(force / 1000).toFixed(1)} kN`, midX - 20, midY - 10);
        ctx.fillText(`(${index + 1})`, midX - 20, midY + 15);
    });

    // Draw nodes
    ctx.fillStyle = 'black';
    nodes.forEach(([x, y], i) => {
        const [tx, ty] = transform(x, y);
        ctx.beginPath();
        ctx.arc(tx, ty, 6, 0, 2 * Math.PI);
        ctx.fill();

        // Label node
        ctx.font = '16px Arial';
        ctx.fillText(String.fromCharCode(65 + i), tx + 10, ty + 5);
    });

    // Simplified legend
    ctx.font = 'bold 14px Arial';
    ctx.fillStyle = 'black';
    ctx.fillText('Legend:', canvas.width - 140, 30);

    ctx.fillStyle = 'blue';
    ctx.fillText('Tension', canvas.width - 140, 50);

    ctx.fillStyle = 'red';
    ctx.fillText('Compression', canvas.width - 140, 70);
}
