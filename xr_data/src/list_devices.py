"""print every device SteamVR sees (class + serial). use it to fill in configs/devices.json."""
import openvr
vr = openvr.init(openvr.VRApplication_Other)
for i in range(openvr.k_unMaxTrackedDeviceCount):
    c = vr.getTrackedDeviceClass(i)
    if c != openvr.TrackedDeviceClass_Invalid:
        s = vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
        print(f"index={i}  class={c}  serial={s}")
openvr.shutdown()
