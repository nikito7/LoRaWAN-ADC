function decodeUplink(input) {
    var data = {};
    var warnings = [];

    if (input.fPort == 70) {
        data.txFail = ((input.bytes[0] << 8)
                 + input.bytes[1]);
        data.ADC = ((input.bytes[2] << 8)
                 + input.bytes[3]);
        data.Volt = (data.ADC / 1024 * 6.0);
    }
    else {
        warnings.push("Unsupported fPort");
    }
    return {
        data: data,
        warnings: warnings
    };
}

// EOF
