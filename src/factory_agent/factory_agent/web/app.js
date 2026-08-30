"use strict";

const elements = {
  activeOrder: document.querySelector("#active-order"),
  batteryBar: document.querySelector("#battery-bar"),
  batteryValue: document.querySelector("#battery-value"),
  clock: document.querySelector("#clock"),
  conversation: document.querySelector("#conversation"),
  finishedCount: document.querySelector("#finished-count"),
  form: document.querySelector("#chat-form"),
  input: document.querySelector("#operator-input"),
  machineList: document.querySelector("#machine-list"),
  modelName: document.querySelector("#model-name"),
  orderDetail: document.querySelector("#order-detail"),
  rawCount: document.querySelector("#raw-count"),
  refreshState: document.querySelector("#refresh-state"),
  sendButton: document.querySelector("#send-button"),
  systemStatus: document.querySelector("#system-status"),
  toolTrace: document.querySelector("#tool-trace"),
  traceCount: document.querySelector("#trace-count"),
};

const uiState = {
  busy: false,
  traceCount: 0,
};

function timeLabel() {
  return new Intl.DateTimeFormat("zh-CN", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  }).format(new Date());
}

function updateClock() {
  elements.clock.textContent = timeLabel();
}

function setConnection(status, label) {
  elements.systemStatus.className = `status-pill status-${status}`;
  elements.systemStatus.lastChild.textContent = label;
}

function setBusy(busy) {
  uiState.busy = busy;
  elements.sendButton.disabled = busy;
  elements.input.disabled = busy;
  document.querySelectorAll("[data-prompt]").forEach((button) => {
    button.disabled = busy;
  });
}

function appendMessage(role, text, error = false) {
  const article = document.createElement("article");
  article.className = `message message-${role}`;
  if (error) {
    article.classList.add("message-error");
  }

  const label = document.createElement("div");
  label.className = "message-label";
  const speaker = document.createElement("span");
  speaker.textContent = role === "user" ? "OPERATOR" : "FACTORY AGENT";
  const timestamp = document.createElement("span");
  timestamp.textContent = timeLabel();
  label.append(speaker, timestamp);

  const content = document.createElement("p");
  content.textContent = text;
  article.append(label, content);
  elements.conversation.append(article);
  elements.conversation.scrollTop = elements.conversation.scrollHeight;
  return article;
}

function appendTyping() {
  const article = document.createElement("article");
  article.className = "message message-agent";
  article.id = "typing-message";

  const label = document.createElement("div");
  label.className = "message-label";
  label.textContent = "FACTORY AGENT · 正在分析工具";

  const typing = document.createElement("div");
  typing.className = "typing";
  typing.setAttribute("aria-label", "Agent 正在回复");
  typing.append(
    document.createElement("span"),
    document.createElement("span"),
    document.createElement("span"),
  );
  article.append(label, typing);
  elements.conversation.append(article);
  elements.conversation.scrollTop = elements.conversation.scrollHeight;
}

function removeTyping() {
  document.querySelector("#typing-message")?.remove();
}

function displayValue(value) {
  if (value === null || value === undefined || value === "") {
    return "—";
  }
  if (Array.isArray(value)) {
    return value.join(", ") || "[]";
  }
  if (typeof value === "object") {
    return JSON.stringify(value);
  }
  return String(value);
}

function normalizedMachineState(machine) {
  const value = machine.state ?? machine.status ?? "UNKNOWN";
  if (typeof value === "number") {
    const states = {
      0: "IDLE",
      1: "READY",
      2: "PROCESSING",
      3: "DONE",
      4: "FAULT",
    };
    return states[value] ?? `STATE_${value}`;
  }
  return String(value).toUpperCase();
}

function renderMachines(machines) {
  if (!Array.isArray(machines) || machines.length === 0) {
    return;
  }
  elements.machineList.replaceChildren();
  machines.forEach((machine, index) => {
    const row = document.createElement("div");
    row.className = "machine-row";

    const identity = document.createElement("div");
    identity.className = "machine-identity";
    const number = document.createElement("span");
    number.className = "machine-number";
    number.textContent = String(index + 1).padStart(2, "0");
    const name = document.createElement("span");
    name.textContent = machine.machine_id ?? machine.id ?? `machine_${index + 1}`;
    identity.append(number, name);

    const state = normalizedMachineState(machine);
    const stateNode = document.createElement("span");
    stateNode.className = `machine-state state-${state.toLowerCase()}`;
    stateNode.textContent = state;
    row.append(identity, stateNode);
    elements.machineList.append(row);
  });
}

function renderFactoryState(data) {
  if (!data || typeof data !== "object") {
    return;
  }
  const inventory = data.inventory ?? {};
  elements.rawCount.textContent = displayValue(
    inventory.raw_parts ?? data.raw_part_count,
  );
  elements.finishedCount.textContent = displayValue(
    inventory.finished_parts ?? data.finished_part_count,
  );

  const battery = data.battery ?? {};
  let percentage = Number(
    battery.percentage ?? battery.percentage_remaining ?? data.battery_percentage,
  );
  if (Number.isFinite(percentage)) {
    if (percentage <= 1) {
      percentage *= 100;
    }
    percentage = Math.max(0, Math.min(100, percentage));
    elements.batteryValue.textContent = `${percentage.toFixed(0)}%`;
    elements.batteryBar.style.width = `${percentage}%`;
  }

  renderMachines(data.machines);
  const orderId = data.active_order_id;
  if (orderId) {
    setActiveOrder(orderId, "ROS 正在执行或跟踪该订单。");
  }
}

function setActiveOrder(orderId, detail) {
  elements.activeOrder.textContent = orderId;
  elements.orderDetail.textContent = detail;
}

function addTrace(execution) {
  if (uiState.traceCount === 0) {
    elements.toolTrace.replaceChildren();
  }
  uiState.traceCount += 1;
  elements.traceCount.textContent = String(uiState.traceCount);

  const payload = execution.payload ?? {};
  const accepted = execution.protocol_succeeded && payload.accepted !== false;
  const card = document.createElement("article");
  card.className = "tool-card";

  const head = document.createElement("div");
  head.className = "tool-head";
  const name = document.createElement("span");
  name.className = "tool-name";
  name.textContent = execution.name ?? "unknown_tool";
  const result = document.createElement("span");
  result.className = `tool-result${accepted ? "" : " rejected"}`;
  result.textContent = accepted ? "ACCEPTED" : "REJECTED";
  head.append(name, result);

  const details = document.createElement("dl");
  const rows = [
    ["参数", execution.arguments],
    ["请求", payload.request_id],
    ["订单", payload.order_id],
    ["说明", payload.message],
  ];
  rows.forEach(([label, value]) => {
    if (value === undefined || value === null || value === "") {
      return;
    }
    const term = document.createElement("dt");
    term.textContent = label;
    const description = document.createElement("dd");
    description.textContent = displayValue(value);
    details.append(term, description);
  });

  card.append(head, details);
  elements.toolTrace.prepend(card);

  if (payload.order_id) {
    setActiveOrder(
      payload.order_id,
      accepted ? "订单已被确定性执行器接受。" : "订单请求被拒绝。",
    );
  }
  renderFactoryState(payload.data);
}

async function sendMessage(message) {
  const normalized = message.trim();
  if (!normalized || uiState.busy) {
    return;
  }
  appendMessage("user", normalized);
  elements.input.value = "";
  setBusy(true);
  appendTyping();

  try {
    const response = await fetch("/api/chat", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({message: normalized}),
    });
    const payload = await response.json();
    removeTyping();
    if (!response.ok) {
      throw new Error(payload.error ?? `HTTP ${response.status}`);
    }
    appendMessage("agent", payload.reply || "工具已执行，但模型没有返回说明。");
    (payload.tool_executions ?? []).forEach(addTrace);
  } catch (error) {
    removeTyping();
    appendMessage(
      "agent",
      `请求未执行：${error.message}`,
      true,
    );
  } finally {
    setBusy(false);
    elements.input.focus();
  }
}

async function loadHealth() {
  try {
    const response = await fetch("/api/health", {cache: "no-store"});
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const health = await response.json();
    elements.modelName.textContent = health.model;
    setConnection(
      health.llm_configured ? "online" : "offline",
      health.llm_configured ? "操作台就绪" : "缺少模型配置",
    );
  } catch (_error) {
    setConnection("offline", "后端不可用");
  }
}

elements.form.addEventListener("submit", (event) => {
  event.preventDefault();
  sendMessage(elements.input.value);
});

elements.input.addEventListener("keydown", (event) => {
  if (event.key === "Enter" && !event.shiftKey) {
    event.preventDefault();
    elements.form.requestSubmit();
  }
});

document.querySelectorAll("[data-prompt]").forEach((button) => {
  button.addEventListener("click", () => {
    sendMessage(button.dataset.prompt);
  });
});

elements.refreshState.addEventListener("click", () => {
  sendMessage("查看机床、库存和电量");
});

updateClock();
setInterval(updateClock, 1000);
loadHealth();
