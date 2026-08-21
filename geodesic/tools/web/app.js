const PIECES = {
  white: { k: "♔", q: "♕", r: "♖", b: "♗", n: "♘", p: "♙" },
  black: { k: "♚", q: "♛", r: "♜", b: "♝", n: "♞", p: "♟" },
};

const state = {
  board: null,
  selected: null,
  legal: [],
};

async function api(path, payload) {
  const options = payload === undefined ? {} : {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  };
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok) {
    throw new Error(data.error || "request failed");
  }
  return data;
}

function setStatus(message, isError = false) {
  const node = document.getElementById("board-status");
  node.textContent = message;
  node.classList.toggle("error", isError);
}

function renderBoard(board) {
  state.board = board;
  document.getElementById("fen").value = board.fen;
  const root = document.getElementById("board");
  root.innerHTML = "";
  for (let displayRow = 7; displayRow >= 0; displayRow -= 1) {
    for (let col = 0; col < 8; col += 1) {
      const square = board.squares.find((item) => item.row === displayRow && item.col === col);
      const button = document.createElement("button");
      button.type = "button";
      button.className = `sq ${(displayRow + col) % 2 === 0 ? "dark" : "lite"}`;
      if (state.selected === square.square) button.classList.add("selected");
      if (state.legal.includes(square.square)) button.classList.add("legal");
      button.textContent = square.piece ? PIECES[square.piece.color][square.piece.kind] : "";
      button.addEventListener("click", () => onSquare(square.square));
      root.appendChild(button);
    }
  }
  const flags = [
    board.side_to_move === "white" ? "دور الأبيض" : "دور الأسود",
    board.in_check ? "كش" : null,
    board.checkmate ? "كش مات" : null,
    board.stalemate ? "تعادل بالاختناق" : null,
  ].filter(Boolean);
  setStatus(flags.join(" · "));
}

function targetsFrom(origin) {
  return (state.board?.legal_moves || [])
    .filter((move) => move.startsWith(origin))
    .map((move) => move.slice(2, 4));
}

async function onSquare(square) {
  if (state.selected && state.legal.includes(square)) {
    const promo = state.board.legal_moves.find((move) => move.startsWith(state.selected + square) && move.length > 4);
    const move = promo || (state.selected + square);
    try {
      const next = await api("/api/apply", { fen: state.board.fen, move });
      state.selected = null;
      state.legal = [];
      renderBoard(next);
    } catch (error) {
      setStatus(error.message, true);
    }
    return;
  }
  state.selected = square;
  state.legal = targetsFrom(square);
  renderBoard(state.board);
}

function renderCandidates(candidates) {
  const root = document.getElementById("candidates");
  if (!candidates?.length) {
    root.innerHTML = "<p class='note'>لا توجد فرضيات بعد.</p>";
    return;
  }
  root.innerHTML = candidates.map((item, index) => `
    <article class="candidate">
      <strong>${index + 1}. ${item.move}</strong>
      <span>score ${item.overall_score.toFixed(3)} · d_g ${item.geodesic_error.toFixed(3)} · pressure ${item.pressure.toFixed(3)}</span>
    </article>
  `).join("");
}

async function loadEnvironment() {
  const env = await api("/api/environment");
  document.getElementById("env-status").textContent = env.machine_status;
  document.getElementById("env-torch").textContent = env.torch_available ? env.pytorch_version : "غير متوفر";
  document.getElementById("env-cuda").textContent = env.cuda_available ? "نعم" : "لا";
}

async function analyze() {
  const fen = document.getElementById("fen").value.trim();
  const metric = document.getElementById("metric").value;
  const k = Number(document.getElementById("topk").value || 3);
  const result = await api("/api/analyze", { fen, metric, k });
  state.selected = null;
  state.legal = [];
  renderBoard(result.board);
  renderCandidates(result.candidates);
  document.getElementById("pipeline-json").textContent = JSON.stringify({
    status: result.status,
    blocked_reason: result.blocked_reason,
    kernel: result.kernel,
    transformer: result.transformer,
    translator_state: result.translator_state,
  }, null, 2);
}

async function resetBoard() {
  const board = await api("/api/start");
  state.selected = null;
  state.legal = [];
  renderBoard(board);
  document.getElementById("candidates").innerHTML = "";
}

async function kernel() {
  const read = (id, name) => ({
    name,
    coordinates: document.getElementById(id).value.split(",").map((value) => Number(value.trim())),
  });
  const metric = document.getElementById("metric").value;
  const report = await api("/api/kernel", { metric, points: [read("p0", "a"), read("p1", "b"), read("p2", "c")] });
  const stats = document.getElementById("kernel-stats");
  stats.innerHTML = `
    <div><dt>الطول</dt><dd>${report.path_length.toFixed(3)}</dd></div>
    <div><dt>التشوه</dt><dd>${report.distortion.toFixed(3)}</dd></div>
  `;
}

document.getElementById("analyze").addEventListener("click", () => analyze().catch((error) => setStatus(error.message, true)));
document.getElementById("reset").addEventListener("click", () => resetBoard().catch((error) => setStatus(error.message, true)));
document.getElementById("kernel").addEventListener("click", () => kernel().catch((error) => setStatus(error.message, true)));

loadEnvironment().catch((error) => setStatus(error.message, true));
resetBoard().catch((error) => setStatus(error.message, true));
