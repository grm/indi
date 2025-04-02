# Discord Notifier for INDI

The Discord Notifier is an INDI auxiliary device that allows you to receive notifications about your astronomy equipment in a Discord channel. It works by snooping on other INDI devices and sending updates to Discord via webhooks.

## Features

- Snoop on any INDI device and property
- Send notifications to Discord when property values change
- Configure which device and properties to monitor
- Enable/disable notifications
- Test notifications

## Setup

### 1. Create a Discord Webhook

1. Open Discord and go to the server where you want to receive notifications
2. Right-click on the channel and select "Edit Channel"
3. Go to "Integrations" > "Webhooks"
4. Click "New Webhook"
5. Give it a name (e.g., "INDI Notifications")
6. Copy the webhook URL (it should look like `https://discord.com/api/webhooks/...`)

### 2. Configure the Discord Notifier

1. Start INDI server with the Discord Notifier driver
2. Connect to the INDI server with your client (e.g., KStars/Ekos)
3. Go to the "Discord" tab in the Discord Notifier device panel
4. Paste the webhook URL in the "Webhook URL" field
5. Enter the name of the device you want to monitor in the "Device" field
6. Enter the properties you want to monitor in the "Properties" fields
7. Click "On" in the "Notifications" section
8. Click "Send Test" to verify that notifications are working

## Example Configuration

To monitor a mount's coordinates:

- Device: "Mount Name"
- Property_1: "EQUATORIAL_EOD_COORD"
- Property_2: "TELESCOPE_INFO"

To monitor a camera's temperature:

- Device: "Camera Name"
- Property_1: "CCD_TEMPERATURE"

## Troubleshooting

- If test notifications fail, check that your webhook URL is correct
- Make sure the device and property names match exactly what's in INDI
- Check that the INDI server has internet access
- If you're using a firewall, make sure outgoing HTTPS connections are allowed

## Building from Source

The Discord Notifier is included in the INDI library. To build it:

```bash
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make
sudo make install
```

## License

This driver is licensed under the GNU General Public License v2.0. 