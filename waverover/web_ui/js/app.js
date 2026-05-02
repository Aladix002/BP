/* global window, React, ReactDOM */
"use strict";

var WR = window.WR;

function App() {
  const { ros, connected, wsUrl, setWsUrl, connect, disconnect } = WR.useRos();
  const [mediaHost, setMediaHost] = React.useState(() => {
    try {
      return localStorage.getItem(WR.ROBOT_MEDIA_HOST_STORAGE_KEY) || "";
    } catch (_) {
      return "";
    }
  });

  return (
    <div className="max-w-[960px] mx-auto px-3 sm:px-5 py-4 pb-10">
      <WR.Header connected={connected} />
      <WR.ConnectionCard
        ros={ros}
        wsUrl={wsUrl}
        setWsUrl={setWsUrl}
        connected={connected}
        connect={connect}
        disconnect={disconnect}
        mediaHost={mediaHost}
        onMediaHostChange={setMediaHost}
      />
      <WR.ModeCard ros={ros} connected={connected} />
      <div className="grid grid-cols-2 gap-2 mb-2">
        <WR.CameraPanel ros={ros} connected={connected} mediaHost={mediaHost} />
        <WR.DrivePanel ros={ros} connected={connected} />
      </div>
      <WR.ImuPanel ros={ros} connected={connected} />
    </div>
  );
}

ReactDOM.createRoot(document.getElementById("root")).render(<App />);
