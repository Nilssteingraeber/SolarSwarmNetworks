import { DataType, Service } from "../models/Service"

export const MockServices: Service[] = [
    {
        name: 'Echo', executable: true, immediateExecute: false, description: "Echo",
        parameters: [{
            name: "Msg",
            iconName: "FaBatteryQuarter",
            readOnly: false,
            inputDataType: DataType.Text,
        },], returns: [{ name: "Echo-Response", outputDataType: DataType.Text }, { name: "Echo-Response", outputDataType: DataType.Text }, { name: "Echo-Response", outputDataType: DataType.Text }, { name: "Echo-Response", outputDataType: DataType.Text }]
    },
    {

        name: 'Drone Status', executable: true, immediateExecute: true, description: "",
        parameters: [
            {
                name: "Battery Saving",
                iconName: "FaBatteryQuarter",
                readOnly: false,
                inputDataType: DataType.Boolean,
            },
            {
                name: "Test Bool",
                iconName: "FaBatteryQuarter",
                readOnly: false,
                inputDataType: DataType.Boolean,
            }, {
                name: "Set Number",
                iconName: "FaBatteryQuarter",
                readOnly: false,
                inputDataType: DataType.Float,
            }, {
                name: "Set Number",
                iconName: "FaBatteryQuarter",
                readOnly: false,
                inputDataType: DataType.Float,
            }, {
                name: "Set Number",
                iconName: "FaBatteryQuarter",
                readOnly: false,
                inputDataType: DataType.Float,
            }]
    },
    {
        name: 'Ping', executable: true, immediateExecute: false, description: "Pings the current Main Server and respons with a long text.",
        parameters: [{
            name: "Set Number",
            iconName: "FaBatteryQuarter",
            readOnly: false,
            inputDataType: DataType.Float,
        },], returns: [{ name: "Ping-Response", outputDataType: DataType.Text }]
    },
];