$featureDesc = Get-Content -Raw -Path "temp_feature_desc.txt"
.\.specify\scripts\powershell\create-new-feature.ps1 -Json -Number 1 -ShortName "physical-ai-book-chapter" $featureDesc
