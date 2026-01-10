{
  "subject": {
    "commonName": {{ toJson .Subject.CommonName }},
    "organization": "ARIA Platform",
    "organizationalUnit": "Root Certificate Authority",
    "country": "US"
  },
  "keyUsage": ["certSign", "crlSign"],
  "basicConstraints": {
    "isCA": true,
    "maxPathLen": 1
  }
}
