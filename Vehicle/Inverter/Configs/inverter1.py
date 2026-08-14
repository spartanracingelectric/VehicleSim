from Vehicle.Inverter.InverterConfig import InverterConfig


inverter1 = InverterConfig(
    max_current_a=700.0,
    con_current_a=300.0,
    max_dc_voltage_v=900.0
)


requested_torque_Nm = 200.0
motor_kt = 0.8

inverter1.current_update(
    requested_torque_Nm,
    motor_kt
)

# // why be boring
print(r"""
        -|=====================|-
       --|  -=[* Inverter *]=- |--
      ---|                     |---
     ----|     -> Torque       |----
    -----|       [*]           |-----
     ----|        |  *math*    |----
      ---|       [*]           |---
       --|        -> AC        |--
        -|=====================|-
""")

print("Requested torque:", requested_torque_Nm, "Nm")
print("Current:", inverter1.current_a, "A")


print("\nFaults:")
print("Overcurrent:", inverter1.has_overcurrent_fault())