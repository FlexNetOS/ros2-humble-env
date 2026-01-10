package service_mesh

# Service-to-service communication policy
default allow_connection = false

# Allow NATS connections from known services
allow_connection {
    input.source.service == "temporal"
    input.destination.service == "nats"
}

allow_connection {
    input.source.service == "n8n"
    input.destination.service == "nats"
}

allow_connection {
    input.source.service == "agixt"
    input.destination.service == "nats"
}
