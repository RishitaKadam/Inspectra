import streamlit as st
import json
import os
import time
import pandas as pd

st.set_page_config(
    page_title="INSPECTRA | Industrial Telemetry",
    page_icon="🏭",
    layout="wide",
    initial_sidebar_state="collapsed"
)

# Custom Industrial CSS Styling
st.markdown("""
    <style>
    .stApp {
        background-color: #0e1117;
        color: #e0e6ed;
    }
    .metric-card {
        background-color: #1a1f2c;
        border: 1px solid #2d3548;
        border-radius: 8px;
        padding: 18px;
        text-align: center;
        box-shadow: 0 4px 6px rgba(0,0,0,0.3);
    }
    .metric-title {
        color: #8b9bb4;
        font-size: 0.85rem;
        font-weight: 600;
        text-transform: uppercase;
        letter-spacing: 1px;
    }
    .metric-value {
        font-size: 2.2rem;
        font-weight: 700;
        margin-top: 5px;
    }
    .status-online {
        color: #00e676;
        font-weight: bold;
    }
    .status-offline {
        color: #ff5252;
        font-weight: bold;
    }
    </style>
""", unsafe_allow_html=True)

data_file = os.path.expanduser('~/inspectra/ros2_ws/telemetry_data.json')

# Header Section
header_col1, header_col2 = st.columns([3, 1])
with header_col1:
    st.title("🏭 INSPECTRA | Autonomous PCB Inspection")
    st.caption("Real-Time ROS 2 Perception & Quality Telemetry")
with header_col2:
    status_container = st.empty()
    time_container = st.empty()

st.markdown("---")

# KPI Metric Cards
c1, c2, c3, c4 = st.columns(4)
m1 = c1.empty()
m2 = c2.empty()
m3 = c3.empty()
m4 = c4.empty()

st.markdown("### 📊 Production Analytics & Audit Trail")
progress_container = st.empty()

col_chart, col_log = st.columns([1, 1])
with col_chart:
    st.subheader("📈 Yield Trend")
    chart_container = st.empty()

with col_log:
    st.subheader("⏱️ Live Timestamped Event Log")
    log_container = st.empty()

# Real-Time Refresh Loop
while True:
    if os.path.exists(data_file):
        try:
            with open(data_file, 'r') as f:
                data = json.load(f)

            # Update Header Telemetry
            status_container.markdown(f"**System Status:** <span class='status-online'>● {data.get('system_status', 'ACTIVE')}</span>", unsafe_allow_html=True)
            time_container.markdown(f"**Last Sync:** `{data.get('last_updated', 'N/A')}`")

            # Update KPI Cards
            m1.markdown(f"""<div class="metric-card"><div class="metric-title">Total Inspected</div><div class="metric-value" style="color:#00b0ff;">{data['total_inspected']}</div></div>""", unsafe_allow_html=True)
            m2.markdown(f"""<div class="metric-card"><div class="metric-title">Pass Count</div><div class="metric-value" style="color:#00e676;">{data['pass_count']}</div></div>""", unsafe_allow_html=True)
            m3.markdown(f"""<div class="metric-card"><div class="metric-title">Fail Count</div><div class="metric-value" style="color:#ff5252;">{data['fail_count']}</div></div>""", unsafe_allow_html=True)
            
            yield_val = data['yield_rate']
            yield_color = "#00e676" if yield_val >= 90 else "#ffab00" if yield_val >= 75 else "#ff5252"
            m4.markdown(f"""<div class="metric-card"><div class="metric-title">Yield Rate</div><div class="metric-value" style="color:{yield_color};">{yield_val}%</div></div>""", unsafe_allow_html=True)

            # Progress Bar for Target Yield (90% Threshold)
            progress_container.progress(min(int(yield_val), 100), text=f"Target Quality Yield: {yield_val}% / 100%")

            # History Log & Charting
            history = data.get("history", [])
            if history:
                df = pd.DataFrame(history)
                log_container.dataframe(
                    df[["time", "status", "detail", "yield_after"]],
                    column_config={
                        "time": "Time",
                        "status": "Result",
                        "detail": "Inspection Note",
                        "yield_after": "Yield %"
                    },
                    use_container_width=True,
                    hide_index=True
                )
                
                # Plot yield over recent events
                df_chart = df.iloc[::-1] # Reverse to chronological order
                chart_container.line_chart(df_chart.set_index("time")["yield_after"])
            else:
                log_container.info("Waiting for first inspection event...")

        except (json.JSONDecodeError, KeyError):
            pass
    else:
        status_container.markdown("**System Status:** <span class='status-offline'>○ WAITING FOR DATA</span>", unsafe_allow_html=True)
        time_container.markdown("**Last Sync:** `No File`")

    time.sleep(1)
