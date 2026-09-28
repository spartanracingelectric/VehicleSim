import pybamm

# Create a Thevenin equivalent-circuit model with one RC element
model = pybamm.equivalent_circuit.Thevenin(
    options={
        "number of rc elements": "1"
    }
)
