{
  "subject": {
    "commonName": {{ toJson .Subject.CommonName }},
    "organization": "ARIA Platform",
    "organizationalUnit": "{{ .Token.service | default "Infrastructure" }}",
    "country": "US"
  },
  "sans": {{ toJson .SANs }},
  "keyUsage": ["keyEncipherment", "digitalSignature"],
  "extKeyUsage": ["serverAuth", "clientAuth"],
  "basicConstraints": {
    "isCA": false,
    "maxPathLen": 0
  },
  "extensions": [
    {
      "id": "1.3.6.1.4.1.99999.1",
      "critical": false,
      "value": "{{ .Token.environment | default "development" }}"
    },
    {
      "id": "1.3.6.1.4.1.99999.2",
      "critical": false,
      "value": "{{ .Token.service | default "unknown" }}"
    }
  ]
}
