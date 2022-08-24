<h1>swag8139</h1>

<h3>Very simple RTL8139 rev.20 Linux driver</h3>
<br>
built this driver cause i want to focus on ML and IoT , so this is i guess my try at doing even more low-level shit.
<br>
it should work propertly, RX works TX works, i just didn't spend time on setting it up on linux, too lazy, sorry.
<br>
<h3>This driver contains :</h3>
<b>interrupt handling rx ,tx
<br>
pci_alloc_consistent instead of dma_alloc_conherent( who gives a damn about struct device ?)
<br>
hardware initilization
<br>
tx initialization
<br><br>

  NOTE to myself :i feel like i need to focus even more on programming , i like when people get mad at me .
<h1>Resources</h1>
this one is a bit problematic.
<br>
i tried to understand linux network stack by reversing 8139too.ko , and it kind of worked.
<br>
also i took a look and 8139cp.ko 
</b>
<h1>dmesg output when i execute - "ifconfig eth0 up"</h1>
<br>

![image](https://user-images.githubusercontent.com/59802817/186494475-93cc2d9a-4373-4d38-af50-80cf9d779776.png)
